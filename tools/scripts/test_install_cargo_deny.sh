#!/usr/bin/env bash
# test_install_cargo_deny.sh: offline self-test for install_cargo_deny.sh.
#
#   tools/scripts/test_install_cargo_deny.sh
#
# `curl` is a PATH shim that serves one fixture tarball per platform by its
# release URL, wrong bytes, or an HTTP error, as the case sets, and logs every
# download it is asked for. `curl --version` reports the version a case sets,
# and a curl older than 7.71 rejects `--retry-all-errors` as an unknown option,
# as a real one does.
#
# The installer under test is a copy whose four pinned checksums are replaced
# by the fixtures' checksums; the real checksums name the real release, which
# this test never downloads. The copy runs under a PATH of the shim and the
# system directories only, so no cargo-deny already on the machine is seen.
#
# Exit 0 = every case held. Nonzero = the first failed case, named on stderr.
set -euo pipefail

script_dir=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)
installer=$script_dir/install_cargo_deny.sh
work=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-cargo-deny-test.XXXXXX")
trap 'rm -rf "$work"' EXIT

fail() {
    printf 'test_install_cargo_deny: FAIL: %s\n' "$1" >&2
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

version=$(sed -n 's/^CARGO_DENY_PINNED_VERSION=//p' "$installer")
[ -n "$version" ] || fail "could not read CARGO_DENY_PINNED_VERSION from $installer"

target_of() {
    case "$1" in
        linux) printf x86_64-unknown-linux-musl ;;
        linux-arm) printf aarch64-unknown-linux-musl ;;
        mac-x86) printf x86_64-apple-darwin ;;
        mac-arm) printf aarch64-apple-darwin ;;
    esac
}
case "$(uname -s):$(uname -m)" in
    Linux:x86_64|Linux:amd64) host_platform=linux ;;
    Linux:aarch64|Linux:arm64) host_platform=linux-arm ;;
    Darwin:x86_64) host_platform=mac-x86 ;;
    Darwin:arm64|Darwin:aarch64) host_platform=mac-arm ;;
    *) fail "no fixture platform for $(uname -s) $(uname -m)" ;;
esac
asset_of() { printf 'cargo-deny-%s-%s.tar.gz' "$version" "$(target_of "$1")"; }
release=https://github.com/EmbarkStudios/cargo-deny/releases/download/$version
host_url=$release/$(asset_of "$host_platform")

# --- fixtures --------------------------------------------------------------
# One good tarball per platform, laid out as the release lays it out, whose
# `cargo-deny` names its platform; and one wrong tarball.
fixtures=$work/fixtures
mkdir -p "$fixtures"
make_tarball() {
    local name=$1 platform=$2 label=$3 dir top
    dir=$(mktemp -d "$work/tarball.XXXXXX")
    top=cargo-deny-$version-$(target_of "$platform")
    mkdir -p "$dir/$top"
    printf '#!/bin/sh\nprintf '"'"'cargo-deny %s (fixture %s)\\n'"'"'\n' "$version" "$label" > "$dir/$top/cargo-deny"
    chmod +x "$dir/$top/cargo-deny"
    COPYFILE_DISABLE=1 tar -czf "$fixtures/$name.tar.gz" -C "$dir" "$top"
    rm -rf "$dir"
}
for platform in linux linux-arm mac-x86 mac-arm; do
    make_tarball "good-$platform" "$platform" "$platform"
done
make_tarball wrong "$host_platform" "wrong bytes"

# The copy differs from the installer in the four checksum lines only.
sed -e "s/^CARGO_DENY_SHA256_linux=.*/CARGO_DENY_SHA256_linux=$(sha256_of "$fixtures/good-linux.tar.gz")/" \
    -e "s/^CARGO_DENY_SHA256_linux_arm=.*/CARGO_DENY_SHA256_linux_arm=$(sha256_of "$fixtures/good-linux-arm.tar.gz")/" \
    -e "s/^CARGO_DENY_SHA256_mac_x86=.*/CARGO_DENY_SHA256_mac_x86=$(sha256_of "$fixtures/good-mac-x86.tar.gz")/" \
    -e "s/^CARGO_DENY_SHA256_mac_arm=.*/CARGO_DENY_SHA256_mac_arm=$(sha256_of "$fixtures/good-mac-arm.tar.gz")/" \
    "$installer" > "$work/install_cargo_deny.sh"
chmod +x "$work/install_cargo_deny.sh"
changed=$(diff "$installer" "$work/install_cargo_deny.sh" | grep -c '^>' || true)
[ "$changed" -eq 4 ] || fail "the fixture copy changed $changed lines, not the 4 checksum lines"

# --- the curl shim ---------------------------------------------------------
# FIXTURE_MODE is `good`, `wrong` or `fail` for every request.
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
for flag in --retry --connect-timeout --max-time --fail; do
    case " $* " in
        *" $flag "*) ;;
        *) printf 'fixture curl: missing %s in: %s\n' "$flag" "$*" >&2; exit 98 ;;
    esac
done
url='' out=''
while [ "$#" -gt 0 ]; do
    case "$1" in
        --url) url=$2; shift 2 ;;
        -o) out=$2; shift 2 ;;
        *) shift ;;
    esac
done
[ -n "$url" ] && [ -n "$out" ] || { printf 'fixture curl: no --url or -o\n' >&2; exit 99; }
platform=''
for candidate in linux:x86_64-unknown-linux-musl linux-arm:aarch64-unknown-linux-musl \
                 mac-x86:x86_64-apple-darwin mac-arm:aarch64-apple-darwin; do
    if [ "$url" = "$FIXTURE_RELEASE/cargo-deny-$FIXTURE_VERSION-${candidate#*:}.tar.gz" ]; then
        platform=${candidate%%:*}
    fi
done
[ -n "$platform" ] || { printf 'fixture curl: no fixture for %s\n' "$url" >&2; exit 97; }
case "$FIXTURE_MODE" in
    good)  cp "$FIXTURE_DIR/good-$platform.tar.gz" "$out" ;;
    wrong) cp "$FIXTURE_DIR/wrong.tar.gz" "$out" ;;
    fail)  printf 'curl: (22) The requested URL returned error: 503\n' >&2; exit 22 ;;
    *)     printf 'fixture curl: unknown mode %s\n' "$FIXTURE_MODE" >&2; exit 96 ;;
esac
SHIM
chmod +x "$shim_bin/curl"

export FIXTURE_DIR=$fixtures FIXTURE_VERSION=$version FIXTURE_RELEASE=$release
# The installer reads this from the environment; a caller's value would point
# the cases at the caller's own cache.
unset CERULION_TOOL_TARBALLS
export FIXTURE_CURL_VERSION=8.7.1 FIXTURE_CURL_OLD=0

# run_case NAME MODE [ARGS...]: run the copy in a fresh CARGO_HOME. Leaves the
# exit status in $status, stdout and stderr together in $out, the download log
# in $log and the CARGO_HOME in $home.
run_case() {
    local name=$1
    export FIXTURE_MODE=$2
    shift 2
    home=$work/$name/cargo
    out=$work/$name/output
    log=$work/$name/urls
    mkdir -p "$work/$name"
    : > "$log"
    status=0
    env PATH="$shim_bin:$home/bin:/usr/bin:/bin" CARGO_HOME="$home" HOME="$work/$name" \
        FIXTURE_LOG="$log" "$work/install_cargo_deny.sh" "$@" > "$out" 2>&1 || status=$?
    export FIXTURE_CURL_VERSION=8.7.1 FIXTURE_CURL_OLD=0
}

urls_requested() { awk '{for (i = 1; i < NF; i++) if ($i == "--url") print $(i + 1)}' "$log"; }
installed_label() { "$home/bin/cargo-deny" --version 2>/dev/null | sed -n 's/.*(fixture \(.*\))$/\1/p'; }

# --- install cases ---------------------------------------------------------

run_case install good
[ "$status" -eq 0 ] || fail "install: exit $status" "$out"
[ "$(installed_label)" = "$host_platform" ] || fail "install: the installed binary is not the $host_platform fixture" "$out"
[ "$(urls_requested)" = "$host_url" ] || fail "install: requested $(urls_requested | tr '\n' ' ')" "$out"
grep -q -- ' --retry-all-errors ' "$log" || fail "install: curl 8.7.1 was not given --retry-all-errors" "$out"
for flag in '--connect-timeout 30' '--max-time 180' '--retry-max-time 120' '--retry 5'; do
    grep -q -- "$flag" "$log" || fail "install: $flag missing" "$out"
done
grep -q '^::' "$out" && fail "install: printed an annotation" "$out"
printf 'the pinned tarball installs: passed\n'

# An already-installed pinned binary is left alone.
mkdir -p "$work/present/cargo/bin"
cp "$work/install/cargo/bin/cargo-deny" "$work/present/cargo/bin/cargo-deny"
run_case present fail
[ "$status" -eq 0 ] || fail "present: exit $status" "$out"
grep -q "cargo-deny $version already on PATH" "$out" || fail "present: not accepted" "$out"
[ ! -s "$log" ] || fail "present: a download was requested" "$out"
printf 'a pinned cargo-deny on PATH is left alone: passed\n'

for curl_version in 7.70.0 7.68.0 7.7 unknown ""; do
    export FIXTURE_CURL_VERSION=$curl_version FIXTURE_CURL_OLD=1
    run_case "old-curl-${curl_version:-empty}" good
    [ "$status" -eq 0 ] || fail "curl '$curl_version': exit $status" "$out"
    [ "$(installed_label)" = "$host_platform" ] || fail "curl '$curl_version': nothing installed" "$out"
    grep -q -- '--retry-all-errors' "$log" && fail "curl '$curl_version': --retry-all-errors was passed" "$out"
    grep -q 'does not report 7.71 or newer, so only its transient errors are retried' "$out" \
        || fail "curl '$curl_version': the fallback was not reported" "$out"
done
export FIXTURE_CURL_VERSION=7.71.0
run_case new-curl-7.71 good
[ "$status" -eq 0 ] || fail "curl 7.71.0: exit $status" "$out"
grep -q -- ' --retry-all-errors ' "$log" || fail "curl 7.71.0: --retry-all-errors was not passed" "$out"
printf 'curl older than 7.71 falls back to plain --retry: passed\n'

run_case wrong-bytes wrong
[ "$status" -ne 0 ] || fail "wrong bytes: exit 0" "$out"
grep -q "^::error::install_cargo_deny: $host_url served bytes that do not match the pinned sha256 of cargo-deny $version ($host_platform)" "$out" \
    || fail "wrong bytes: no error annotation naming the URL" "$out"
[ ! -e "$home/bin/cargo-deny" ] || fail "wrong bytes: a binary was installed" "$out"
printf 'wrong bytes are an error and install nothing: passed\n'

run_case download-fails fail
[ "$status" -ne 0 ] || fail "download fails: exit 0" "$out"
grep -q "download failed: $host_url" "$out" || fail "download fails: the URL was not named" "$out"
[ ! -e "$home/bin/cargo-deny" ] || fail "download fails: a binary was installed" "$out"
printf 'a failed download installs nothing: passed\n'

# --- the tarball cache -------------------------------------------------------

mkdir -p "$work/cache-hit/tarballs"
cp "$fixtures/good-$host_platform.tar.gz" "$work/cache-hit/tarballs/$(asset_of "$host_platform")"
export CERULION_TOOL_TARBALLS=$work/cache-hit/tarballs
run_case cache-hit fail
[ "$status" -eq 0 ] || fail "cache hit: exit $status" "$out"
[ "$(installed_label)" = "$host_platform" ] || fail "cache hit: the cached tarball was not installed" "$out"
[ ! -s "$log" ] || fail "cache hit: a download was requested" "$out"
grep -q '^::' "$out" && fail "cache hit: printed an annotation" "$out"
printf 'a verified cached tarball installs without a download: passed\n'

mkdir -p "$work/cache-wrong/tarballs"
cp "$fixtures/wrong.tar.gz" "$work/cache-wrong/tarballs/$(asset_of "$host_platform")"
export CERULION_TOOL_TARBALLS=$work/cache-wrong/tarballs
run_case cache-wrong good
[ "$status" -eq 0 ] || fail "cache wrong bytes: exit $status" "$out"
[ "$(installed_label)" = "$host_platform" ] || fail "cache wrong bytes: the download was not installed" "$out"
grep -q "^::error::install_cargo_deny: the tarball cache file $work/cache-wrong/tarballs/$(asset_of "$host_platform") does not match the pinned sha256" "$out" \
    || fail "cache wrong bytes: no error annotation naming the cache file" "$out"
[ "$(sha256_of "$work/cache-wrong/tarballs/$(asset_of "$host_platform")")" = "$(sha256_of "$fixtures/good-$host_platform.tar.gz")" ] \
    || fail "cache wrong bytes: the cache file was not replaced" "$out"
printf 'a cached tarball that fails the checksum is an error and is replaced: passed\n'

mkdir -p "$work/cache-wrong-nosource/tarballs"
cp "$fixtures/wrong.tar.gz" "$work/cache-wrong-nosource/tarballs/$(asset_of "$host_platform")"
export CERULION_TOOL_TARBALLS=$work/cache-wrong-nosource/tarballs
run_case cache-wrong-nosource fail
[ "$status" -ne 0 ] || fail "cache wrong bytes, download fails: exit 0" "$out"
[ ! -e "$work/cache-wrong-nosource/tarballs/$(asset_of "$host_platform")" ] \
    || fail "cache wrong bytes, download fails: the cache file that failed the checksum was kept" "$out"
printf 'a cached tarball that fails the checksum is deleted: passed\n'

export CERULION_TOOL_TARBALLS=\~/tarballs
run_case cache-miss good
[ "$status" -eq 0 ] || fail "cache miss: exit $status" "$out"
[ "$(sha256_of "$work/cache-miss/tarballs/$(asset_of "$host_platform")")" = "$(sha256_of "$fixtures/good-$host_platform.tar.gz")" ] \
    || fail "cache miss: the download was not stored under \$HOME/tarballs" "$out"
printf 'a downloaded tarball is stored in the cache: passed\n'

export CERULION_TOOL_TARBALLS=$work/fill/tarballs
run_case fill good --fill-cache linux mac-arm
[ "$status" -eq 0 ] || fail "fill: exit $status" "$out"
for platform in linux mac-arm; do
    [ "$(sha256_of "$work/fill/tarballs/$(asset_of "$platform")")" = "$(sha256_of "$fixtures/good-$platform.tar.gz")" ] \
        || fail "fill: $platform is not in the cache" "$out"
done
[ "$(urls_requested | wc -l | tr -d ' ')" -eq 2 ] || fail "fill: expected two requests" "$out"
[ ! -e "$home/bin" ] || fail "fill: something was installed" "$out"
run_case fill-warm fail --fill-cache linux mac-arm
[ "$status" -eq 0 ] || fail "fill, warm cache: exit $status" "$out"
[ ! -s "$log" ] || fail "fill, warm cache: a download was requested" "$out"
printf 'fill mode fetches the named platforms into the cache: passed\n'

export CERULION_TOOL_TARBALLS=$work/fill-fail/tarballs
run_case fill-fail fail --fill-cache linux
[ "$status" -ne 0 ] || fail "fill, download fails: exit 0" "$out"
[ -z "$(ls -A "$work/fill-fail/tarballs" 2>/dev/null)" ] || fail "fill, download fails: a file was left in the cache" "$out"
: > "$work/not-a-directory"
export CERULION_TOOL_TARBALLS=$work/not-a-directory
run_case fill-unwritable good --fill-cache linux
[ "$status" -ne 0 ] || fail "fill, unwritable cache: exit 0" "$out"
grep -q "could not write $work/not-a-directory/$(asset_of linux)" "$out" || fail "fill, unwritable cache: not diagnosed" "$out"
run_case fill-badplatform good --fill-cache windows
[ "$status" -ne 0 ] || fail "fill: an unknown platform was accepted" "$out"
[ ! -s "$log" ] || fail "fill: an unknown platform fetched something" "$out"
run_case fill-noplatform good --fill-cache
[ "$status" -ne 0 ] || fail "fill: no platform was accepted" "$out"
unset CERULION_TOOL_TARBALLS
run_case fill-nodir good --fill-cache linux
[ "$status" -ne 0 ] || fail "fill: no CERULION_TOOL_TARBALLS was accepted" "$out"
[ ! -s "$log" ] || fail "fill: fetched without a cache directory" "$out"
run_case unknown-arg good --install
[ "$status" -ne 0 ] || fail "an unknown argument was accepted" "$out"
printf 'fill mode and the installer refuse bad arguments: passed\n'

printf 'test_install_cargo_deny: all cases passed\n'
