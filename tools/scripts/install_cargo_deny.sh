#!/usr/bin/env bash
# install_cargo_deny.sh: put a PINNED `cargo-deny` on PATH.
#
#   tools/scripts/install_cargo_deny.sh
#   CERULION_TOOL_TARBALLS=DIR tools/scripts/install_cargo_deny.sh --fill-cache PLATFORM...
#
# Fetches the official pre-built tarball from cargo-deny's GitHub release and
# checks it against the SHA-256 pinned below before anything is extracted. The
# pinned version and its checksums are ONE fact, edited together; the release
# publishes one `.sha256` file per tarball, so moving the pin is
#
#   v=<version>
#   for t in x86_64-unknown-linux-musl aarch64-unknown-linux-musl \
#            x86_64-apple-darwin aarch64-apple-darwin; do
#       curl -fsSL "https://github.com/EmbarkStudios/cargo-deny/releases/download/$v/cargo-deny-$v-$t.tar.gz.sha256"
#   done
#
# TARBALL CACHE. When CERULION_TOOL_TARBALLS names a directory, a tarball of the
# pinned release found there is used without a download, after the same
# checksum test a download faces; a file there that fails it is an `::error::`
# annotation, is deleted, and the release is downloaded. A downloaded tarball is
# copied into the directory. `--fill-cache PLATFORM...` downloads only into
# that directory, one tarball per named platform (`linux`, `linux-arm`,
# `mac-x86`, `mac-arm`), and installs nothing. A leading `~/` in the variable is
# read as $HOME.
#
# Each download attempt is bounded and retried: `--retry 5 --retry-delay 5`
# retries curl's transient class (a timeout, and HTTP 408, 429, 500, 502, 503
# and 504), and `--retry-all-errors` widens that to every failure, a connection
# reset included. That option exists from curl 7.71.0 and an older curl rejects
# it before any transfer starts, so it is passed only when `curl --version`
# reports 7.71 or newer; a version that does not parse counts as older.
# `--connect-timeout` and `--max-time` bound one attempt, which turns a stalled
# connection into a timeout and so into a retry; `--retry-max-time` starts no
# new attempt after 120 s.
#
# Idempotent: a `cargo-deny` of the pinned version already on PATH is left
# alone.
#
# Exit 0 = a `cargo-deny` of CARGO_DENY_PINNED_VERSION is on PATH (or, with
# --fill-cache, every named tarball is in the cache). Nonzero = it is not.

set -euo pipefail

# The pinned version and ITS checksums are ONE fact, edited together.
CARGO_DENY_PINNED_VERSION=0.20.2
CARGO_DENY_SHA256_linux=9f12ed4c49936e09b48bf862b595cde2fe64fcbd9d74dfacac6131ca824c8d5f
CARGO_DENY_SHA256_linux_arm=995c82be0defc7a025cae49a2aa2644ce8245c9a3318fc4103907c6a285e8c7d
CARGO_DENY_SHA256_mac_x86=248da7f581724e470071990c088ffc55c811981715f4cbdb258621fb79f8b7a6
CARGO_DENY_SHA256_mac_arm=fe67d82a10d8597a3549364cb733a3f9cc1bfff9031b7ae46384a9f2a72090c3

version=$CARGO_DENY_PINNED_VERSION
usage="usage: install_cargo_deny.sh [--fill-cache PLATFORM...]"

die() {
    printf 'install_cargo_deny: %s\n' "$1" >&2
    exit 1
}

# annotate LEVEL MESSAGE prints a GitHub Actions workflow command, which the
# runner turns into an annotation of that level on the run; outside Actions it
# is one more log line. `%`, CR and LF are escaped the way the workflow command
# syntax requires. An annotation does not change the exit status.
annotate() {
    local level=$1 message=$2
    message=${message//'%'/%25}
    message=${message//$'\r'/%0D}
    message=${message//$'\n'/%0A}
    printf '::%s::install_cargo_deny: %s\n' "$level" "$message" >&2
}

# target_of PLATFORM: the Rust target triple the release names the tarball by.
target_of() {
    case "$1" in
        linux)     printf '%s' x86_64-unknown-linux-musl ;;
        linux-arm) printf '%s' aarch64-unknown-linux-musl ;;
        mac-x86)   printf '%s' x86_64-apple-darwin ;;
        mac-arm)   printf '%s' aarch64-apple-darwin ;;
        *)         return 1 ;;
    esac
}

# expected_sha_of PLATFORM: the pinned checksum of PLATFORM's tarball.
expected_sha_of() {
    case "$1" in
        linux)     printf '%s' "$CARGO_DENY_SHA256_linux" ;;
        linux-arm) printf '%s' "$CARGO_DENY_SHA256_linux_arm" ;;
        mac-x86)   printf '%s' "$CARGO_DENY_SHA256_mac_x86" ;;
        mac-arm)   printf '%s' "$CARGO_DENY_SHA256_mac_arm" ;;
        *)         return 1 ;;
    esac
}

require_platform() {
    target_of "$1" >/dev/null \
        || die "no pinned tarball is known for platform '$1'; the platforms are linux, linux-arm, mac-x86 and mac-arm"
}

asset_of() {
    printf 'cargo-deny-%s-%s.tar.gz' "$version" "$(target_of "$1")"
}

url_of() {
    printf 'https://github.com/EmbarkStudios/cargo-deny/releases/download/%s/%s' "$version" "$(asset_of "$1")"
}

# VERIFY BEFORE EXTRACTING: the checksum is the last moment the download is
# still just bytes.
require_sha256() {
    command -v shasum >/dev/null 2>&1 || command -v sha256sum >/dev/null 2>&1 \
        || die "neither shasum nor sha256sum is available, so the download cannot be verified; refusing to run it"
}
sha256_of() {
    if command -v sha256sum >/dev/null 2>&1; then
        sha256sum "$1" | cut -d' ' -f1
    else
        shasum -a 256 "$1" | cut -d' ' -f1
    fi
}

curl_retries_all_errors() {
    local major minor
    major=${1%%.*}
    minor=${1#*.}
    minor=${minor%%.*}
    case "$major" in ''|*[!0-9]*) return 1 ;; esac
    case "$minor" in ''|*[!0-9]*) return 1 ;; esac
    [ "$major" -gt 7 ] || { [ "$major" -eq 7 ] && [ "$minor" -ge 71 ]; }
}

# retry_all_errors holds `--retry-all-errors` or nothing; curl_checked records
# that `curl --version` was read, which happens once, before the first download.
retry_all_errors=()
curl_checked=0
check_curl() {
    local curl_version
    [ "$curl_checked" -eq 0 ] || return 0
    curl_checked=1
    curl_version=$(curl --version 2>/dev/null | awk 'NR == 1 {print $2}') || curl_version=""
    if curl_retries_all_errors "$curl_version"; then
        retry_all_errors=(--retry-all-errors)
    else
        printf 'install_cargo_deny: curl %s does not report 7.71 or newer, so only its transient errors are retried\n' \
            "${curl_version:-(version unknown)}" >&2
    fi
}

# The tarball cache directory, empty when CERULION_TOOL_TARBALLS is unset. A
# cached tarball carries its release asset name; CACHE_LAYOUT in
# tool_tarball_cache.sh names that layout and changes with it.
cache_dir=${CERULION_TOOL_TARBALLS:-}
case "$cache_dir" in
    \~/*) cache_dir=$HOME/${cache_dir#\~/} ;;
esac

# store_in_cache FILE PLATFORM: copy FILE into the tarball cache under a
# temporary name and rename it into place, so a reader never sees half a file.
store_in_cache() {
    local cached partial
    cached="$cache_dir/$(asset_of "$2")"
    partial="$cached.partial.$$"
    mkdir -p "$cache_dir" && cp "$1" "$partial" && mv -f "$partial" "$cached" && return 0
    rm -f "$partial"
    return 1
}

# deliver PLATFORM OUT: put the pinned tarball of PLATFORM in OUT, from the
# tarball cache, else from the release. Exits the script when neither delivers
# the pinned bytes.
deliver() {
    local platform=$1 out=$2 want url cached actual
    want=$(expected_sha_of "$platform")
    url=$(url_of "$platform")
    if [ -n "$cache_dir" ]; then
        cached="$cache_dir/$(asset_of "$platform")"
        if [ -f "$cached" ]; then
            actual=$(sha256_of "$cached")
            if [ "$actual" = "$want" ]; then
                cp "$cached" "$out"
                printf 'install_cargo_deny: sha256 %s verified (tarball cache %s)\n' "$actual" "$cached"
                return 0
            fi
            annotate error "the tarball cache file $cached does not match the pinned sha256 of cargo-deny $version ($platform); deleting it and downloading"
            rm -f "$cached"
        fi
    fi
    check_curl
    printf 'install_cargo_deny: fetching %s\n' "$url" >&2
    rm -f "$out"
    curl --proto '=https' --proto-redir '=https' --tlsv1.2 \
        --fail --silent --show-error --location \
        --retry 5 --retry-delay 5 ${retry_all_errors[@]+"${retry_all_errors[@]}"} --retry-max-time 120 \
        --connect-timeout 30 --max-time 180 --url "$url" -o "$out" \
        || { rm -f "$out"; die "download failed: $url"; }
    actual=$(sha256_of "$out")
    if [ "$actual" != "$want" ]; then
        rm -f "$out"
        annotate error "$url served bytes that do not match the pinned sha256 of cargo-deny $version ($platform)"
        die "checksum mismatch for $url
  expected $want
  actual   $actual
The bytes are not the release this repo pinned: do not run them."
    fi
    printf 'install_cargo_deny: sha256 %s verified\n' "$actual"
    if [ -n "$cache_dir" ] && ! store_in_cache "$out" "$platform"; then
        printf 'install_cargo_deny: could not write the tarball cache %s\n' "$cache_dir" >&2
    fi
    return 0
}

mode=install
fill_platforms=()
case "${1-}" in
    --fill-cache)
        [ "$#" -ge 2 ] || die "$usage"
        mode=fill
        shift
        fill_platforms=("$@")
        ;;
    *)
        [ "$#" -eq 0 ] || die "$usage"
        ;;
esac

if [ "$mode" = fill ]; then
    [ -n "$cache_dir" ] || die "--fill-cache writes into CERULION_TOOL_TARBALLS, which is not set"
    for platform in "${fill_platforms[@]}"; do
        require_platform "$platform"
    done
    require_sha256
    tmp=$(mktemp -d)
    trap 'rm -rf "$tmp"' EXIT
    for platform in "${fill_platforms[@]}"; do
        deliver "$platform" "$tmp/$platform.tar.gz"
        cached="$cache_dir/$(asset_of "$platform")"
        if ! { [ -f "$cached" ] && [ "$(sha256_of "$cached")" = "$(expected_sha_of "$platform")" ]; }; then
            die "could not write $cached"
        fi
        printf 'install_cargo_deny: %s holds the pinned %s tarball\n' "$cached" "$platform"
    done
    exit 0
fi

# `cargo-deny --version` prints `cargo-deny 0.20.2`; the version is compared as
# a whole field.
installed_version() {
    command -v cargo-deny >/dev/null 2>&1 || return 1
    cargo-deny --version 2>/dev/null | head -1 | awk '{print $2}'
}
if [ "$(installed_version || true)" = "$version" ]; then
    printf 'install_cargo_deny: cargo-deny %s already on PATH (%s)\n' "$version" "$(command -v cargo-deny)"
    exit 0
fi

case "$(uname -s)" in
    Linux)
        case "$(uname -m)" in
            x86_64|amd64)  platform=linux ;;
            aarch64|arm64) platform=linux-arm ;;
            *) die "unsupported Linux architecture '$(uname -m)'; add it here or install cargo-deny by hand" ;;
        esac
        ;;
    Darwin)
        case "$(uname -m)" in
            x86_64|amd64)  platform=mac-x86 ;;
            aarch64|arm64) platform=mac-arm ;;
            *) die "unsupported macOS architecture '$(uname -m)'; add it here or install cargo-deny by hand" ;;
        esac
        ;;
    *) die "unsupported OS '$(uname -s)'; add it here or install cargo-deny by hand" ;;
esac

require_sha256
dest="${CARGO_HOME:-$HOME/.cargo}/bin"
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
tarball="$tmp/cargo-deny.tar.gz"
deliver "$platform" "$tarball"

member="cargo-deny-$version-$(target_of "$platform")/cargo-deny"
tar -xzf "$tarball" -C "$tmp" "$member" || die "the tarball did not contain $member"
mkdir -p "$dest"
chmod +x "$tmp/$member"
mv "$tmp/$member" "$dest/cargo-deny"

# The binary just written, by path, then PATH: two different failures.
reported=$("$dest/cargo-deny" --version 2>/dev/null | head -1 || true)
[ -n "$reported" ] || die "the installed binary at $dest/cargo-deny does not run"
[ "$(printf '%s' "$reported" | awk '{print $2}')" = "$version" ] \
    || die "the installed binary reports '$reported', not cargo-deny $version"
printf 'install_cargo_deny: installed %s -> %s\n' "$reported" "$dest/cargo-deny"
[ "$(installed_version || true)" = "$version" ] \
    || die "installed $dest/cargo-deny, but '$dest' is not on PATH (or another cargo-deny shadows it)"
