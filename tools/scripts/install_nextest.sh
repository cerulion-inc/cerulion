#!/usr/bin/env bash
# install_nextest.sh — put a PINNED `cargo-nextest` on PATH.
#
# WHY A SCRIPT AND NOT AN ACTION. This repo does not take third-party
# GitHub Actions for tool installs (the SHA-pinned exceptions in ci.yml are
# deliberate and few), and `cargo install cargo-nextest --locked` builds the
# tool from source — minutes per job, on every job, for a binary upstream
# already publishes. So this fetches the OFFICIAL pre-built tarball from
# nextest's own domain, which is the install route nextest documents for CI.
# get.nexte.st answers with a redirect to the asset on nextest's GitHub release;
# when get.nexte.st fails to deliver, or delivers bytes that do not match the
# pin, the script fetches that same asset from the GitHub release directly, and
# the one pinned checksum per platform below verifies the bytes from either
# route.
#
# TARBALL CACHE. When CERULION_TOOL_TARBALLS names a directory, a tarball of the
# pinned release found there is used without a download, after the same
# checksum test a download faces; a file there that fails it is an `::error::`
# annotation, is deleted, and the routes are tried. A tarball a route delivers
# is copied into the directory. `--fill-cache PLATFORM...` downloads only into
# that directory, one tarball per named platform (`linux`, `linux-arm`, `mac`),
# and installs nothing. A leading `~/` in the variable is read as $HOME.
#
# PINNED, never `latest`, and pinned BY CONTENT as well as by version. The
# whole point of the fence in `.config/nextest.toml` is that the runner's
# behaviour is pinned deliberately; a floating version would let an upstream
# release change how tests are scheduled without a commit in this repo. A
# version pin alone fixes the URL, not the bytes — so each platform's tarball
# carries a SHA-256 here, in the same spirit as the SHA-pinned actions in
# ci.yml. The pinned version and its three checksums are ONE fact, edited
# together — a `NEXTEST_VERSION` override that disagrees with the pin is
# refused UP FRONT rather than left to fail later as a puzzling hash mismatch.
# Moving the pin means editing NEXTEST_PINNED_VERSION and regenerating all
# three hashes, which is the reviewable act:
#
#   tools/scripts/install_nextest.sh --print-pins <version>
#
# downloads each platform's tarball from BOTH routes, get.nexte.st and the
# GitHub release asset, and fails naming the platform when either route fails
# or the two routes serve different bytes. Only when every platform agrees does
# it print the version line and the three checksum lines, ready to paste over
# the pin block below. It installs nothing.
#
# Idempotent: an already-present binary of the right version is left alone, so
# a self-hosted runner does not re-download on every job.
#
# Exit 0 = a `cargo nextest` of NEXTEST_VERSION is on PATH. Nonzero = it is
# not, loudly — never a silent fallback to whatever happens to be installed,
# because that is how a pinned toolchain quietly stops being pinned.

set -euo pipefail

# The pinned version and ITS checksums are ONE fact, edited together.
NEXTEST_PINNED_VERSION=0.9.137
NEXTEST_SHA256_linux=38fd6275e111b200bbbed1bd2ae91cbb0d7edd28504879875cff2b3d96f3f311
NEXTEST_SHA256_linux_arm=8f878903d63a69dd0a8fba65d0dc871043423bd0591076f688023b862d58d1a6
NEXTEST_SHA256_mac=94e89c20b233c29c042683e885131d76abf95e795c355a4fe7d64d4b128b90be

NEXTEST_VERSION="${NEXTEST_VERSION:-$NEXTEST_PINNED_VERSION}"

die() {
    printf 'install_nextest: %s\n' "$1" >&2
    exit 1
}

usage="usage: install_nextest.sh [--print-pins VERSION | --fill-cache PLATFORM...]"

# annotate LEVEL MESSAGE prints a GitHub Actions workflow command, which the
# runner turns into an annotation of that level (`warning` or `error`) on the
# run; outside Actions it is one more log line. `%`, CR and LF are escaped the
# way the workflow command syntax requires. An annotation does not change the
# exit status.
annotate() {
    local level=$1 message=$2
    message=${message//'%'/%25}
    message=${message//$'\r'/%0D}
    message=${message//$'\n'/%0A}
    printf '::%s::install_nextest: %s\n' "$level" "$message" >&2
}

# VERIFY BEFORE EXTRACTING. Everything after the checksum runs code that came
# off the network, so the checksum is the last moment it is still just bytes.
# require_sha256 runs before the first download of either mode.
require_sha256() {
    command -v shasum >/dev/null 2>&1 || command -v sha256sum >/dev/null 2>&1 \
        || die "neither shasum nor sha256sum is available, so the download cannot be verified; refusing to run it"
}
sha256_of() {
    if command -v shasum >/dev/null 2>&1; then
        shasum -a 256 "$1" | cut -d' ' -f1
    else
        sha256sum "$1" | cut -d' ' -f1
    fi
}

# Two routes to the SAME bytes, tried in order: get.nexte.st, then the asset
# on nextest's GitHub release that get.nexte.st redirects to. A platform's
# GitHub asset is named by its Rust target triple; get.nexte.st names it by
# the short platform name the checksum variables above use.
target_of() {
    case "$1" in
        linux)     printf '%s' x86_64-unknown-linux-gnu ;;
        linux-arm) printf '%s' aarch64-unknown-linux-gnu ;;
        mac)       printf '%s' universal-apple-darwin ;;
        *)         die "no release target is known for platform '$1'" ;;
    esac
}

# primary_url_of VERSION PLATFORM, fallback_url_of VERSION PLATFORM
primary_url_of() {
    printf 'https://get.nexte.st/%s/%s' "$1" "$2"
}
fallback_url_of() {
    printf 'https://github.com/nextest-rs/nextest/releases/download/cargo-nextest-%s/cargo-nextest-%s-%s.tar.gz' \
        "$1" "$1" "$(target_of "$2")"
}

# download URL FILE: fetch URL into FILE. Returns 0 when curl delivered the
# whole transfer; otherwise removes FILE and returns 1. The bytes are not
# checked here; every caller hashes them.
#
# `--fail` makes an HTTP error a failed download, not a tarball made of an
# error page. `--location` follows get.nexte.st's redirect to the release
# asset, and `--proto-redir '=https'` keeps every hop of it on HTTPS.
#
# `--retry 5 --retry-delay 5` retries curl's transient class (a timeout, and
# HTTP 408, 429, 500, 502, 503 and 504) five times at least five seconds apart,
# longer when the server sends a Retry-After; curl discards a partial file
# before each retry. `--retry-all-errors` widens that to every failure,
# including a connection reset (`curl: (35) Recv failure: Connection reset by
# peer`), which `--retry` alone gives up on. It exists from curl 7.71.0 and an
# older curl rejects the option before any transfer starts, so it is passed only
# when `curl --version` reports 7.71 or newer; a version that does not parse
# counts as older. `--connect-timeout` and `--max-time` bound one attempt, and
# `--retry-max-time` starts no new attempt after 120 s, so a host that stalls
# hands over to the next route.
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
    local version
    [ "$curl_checked" -eq 0 ] || return 0
    curl_checked=1
    version=$(curl --version 2>/dev/null | awk 'NR == 1 {print $2}') || version=""
    if curl_retries_all_errors "$version"; then
        retry_all_errors=(--retry-all-errors)
    else
        printf 'install_nextest: curl %s does not report 7.71 or newer, so only its transient errors are retried\n' \
            "${version:-(version unknown)}" >&2
    fi
}

download() {
    local src=$1 out=$2
    check_curl
    printf 'install_nextest: fetching %s\n' "$src" >&2
    rm -f "$out"
    curl --proto '=https' --proto-redir '=https' --tlsv1.2 \
        --fail --silent --show-error --location \
        --retry 5 --retry-delay 5 ${retry_all_errors[@]+"${retry_all_errors[@]}"} --retry-max-time 120 \
        --connect-timeout 30 --max-time 180 --url "$src" -o "$out" && return 0
    printf 'install_nextest: download failed for %s\n' "$src" >&2
    rm -f "$out"
    return 1
}

# print_pins VERSION is the pin-bump recipe. Every platform's tarball is
# fetched from both routes and the two hashes compared; the paste-ready lines
# go to stdout only after all three platforms agree, so a failure prints no
# partial pin block.
print_pins() {
    local version=$1 platform primary fallback primary_sha fallback_sha pins
    case "$version" in
        ''|[!0-9]*|*[!0-9A-Za-z.-]*)
            die "--print-pins takes a release version such as $NEXTEST_PINNED_VERSION, not '$version'" ;;
    esac
    require_sha256
    pins_tmp=$(mktemp -d)
    trap 'rm -rf "$pins_tmp"' EXIT
    pins="NEXTEST_PINNED_VERSION=$version"
    for platform in linux linux-arm mac; do
        primary=$(primary_url_of "$version" "$platform")
        fallback=$(fallback_url_of "$version" "$platform")
        download "$primary" "$pins_tmp/$platform.primary" \
            || die "$platform: get.nexte.st did not serve $primary"
        download "$fallback" "$pins_tmp/$platform.fallback" \
            || die "$platform: the GitHub release did not serve $fallback.
If the release renamed its assets, fallback_url_of in this script must follow
the new names before the pin moves; the install fallback would fail the same way."
        primary_sha=$(sha256_of "$pins_tmp/$platform.primary")
        fallback_sha=$(sha256_of "$pins_tmp/$platform.fallback")
        if [ "$primary_sha" != "$fallback_sha" ]; then
            die "$platform: the two routes serve different bytes, so no one checksum can pin both:
  $primary
    sha256 $primary_sha
  $fallback
    sha256 $fallback_sha"
        fi
        pins="$pins
NEXTEST_SHA256_${platform//-/_}=$primary_sha"
    done
    printf '%s\n' "$pins"
}

mode=install
fill_platforms=()
case "${1-}" in
    --print-pins)
        [ "$#" -eq 2 ] || die "$usage"
        print_pins "$2"
        exit 0
        ;;
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

# NEXTEST_VERSION selects the download; the checksums above do not follow it.
# Left alone, that combination reads as a usable knob and behaves as a trap: the
# override changes the URL, the download succeeds, and the run dies on a
# checksum mismatch that looks like a corrupted or tampered artifact rather than
# like "you changed the version and not the hashes".
#
# So the mismatch is refused HERE, before anything is fetched, naming both
# versions. The override stays — it is genuinely useful for trying a release
# before pinning it — but it now tells you the truth immediately instead of
# after a download and a confusing hash error.
if [ "$NEXTEST_VERSION" != "$NEXTEST_PINNED_VERSION" ]; then
    die "NEXTEST_VERSION is '$NEXTEST_VERSION' but the checksums in this script are
pinned to '$NEXTEST_PINNED_VERSION', so the download could not be verified.

A version and its checksums are ONE edit. To move the pin, change
NEXTEST_PINNED_VERSION and regenerate all three hashes (the header shows how),
then re-read the fence semantics in .config/nextest.toml — an upstream release
can change how tests are SCHEDULED, which is the whole reason this is pinned."
fi

# expected_sha_of PLATFORM: the pinned checksum of PLATFORM's tarball.
expected_sha_of() {
    case "$1" in
        linux)     printf '%s' "$NEXTEST_SHA256_linux" ;;
        linux-arm) printf '%s' "$NEXTEST_SHA256_linux_arm" ;;
        mac)       printf '%s' "$NEXTEST_SHA256_mac" ;;
        *)         die "no pinned checksum is known for platform '$1'; the platforms are linux, linux-arm and mac" ;;
    esac
}

# The tarball cache directory, empty when CERULION_TOOL_TARBALLS is unset. A
# cached tarball carries its GitHub release asset name.
cache_dir=${CERULION_TOOL_TARBALLS:-}
case "$cache_dir" in
    \~/*) cache_dir=$HOME/${cache_dir#\~/} ;;
esac
cached_path_of() {
    printf '%s/cargo-nextest-%s-%s.tar.gz' "$cache_dir" "$NEXTEST_VERSION" "$(target_of "$1")"
}

# fetch_verified URL OUT EXPECTED: download URL to OUT and compare it with
# EXPECTED. Returns 0 only when OUT holds those bytes. Otherwise it deletes OUT,
# prints the reason on stderr and returns 1 when the download failed, 2 when the
# bytes arrived and do not match.
fetch_verified() {
    local src=$1 out=$2 want=$3 actual
    download "$src" "$out" || return 1
    actual=$(sha256_of "$out")
    if [ "$actual" != "$want" ]; then
        printf 'install_nextest: checksum mismatch for %s\n  expected %s\n  actual   %s\n' \
            "$src" "$want" "$actual" >&2
        rm -f "$out"
        return 2
    fi
    printf 'install_nextest: sha256 %s verified\n' "$actual"
}

# store_in_cache FILE PLATFORM: copy FILE into the tarball cache under a
# temporary name and rename it into place, so a reader never sees half a file.
# Returns nonzero when the directory cannot be written.
store_in_cache() {
    local cached partial
    cached=$(cached_path_of "$2")
    partial="$cached.partial.$$"
    mkdir -p "$cache_dir" && cp "$1" "$partial" && mv -f "$partial" "$cached" && return 0
    rm -f "$partial"
    return 1
}

# deliver PLATFORM PRIMARY OUT: put the pinned tarball of PLATFORM in OUT, from
# the tarball cache, then PRIMARY, then the GitHub release asset. Exits the
# script when no source delivers it.
#
# A source that fails hands over to the next one, whose bytes face the same
# pinned checksum, and the failures are reported at different levels: a failed
# download (an outage, a 404) as a `warning`; bytes that arrived, or that the
# cache held, and do not match the pinned checksum as an `error`. The error
# annotation stays on the run's summary when a later source then delivers and
# the job passes.
deliver() {
    local platform=$1 primary=$2 out=$3 want fallback cached actual primary_status fallback_status
    want=$(expected_sha_of "$platform")
    fallback=$(fallback_url_of "$NEXTEST_VERSION" "$platform")
    if [ -n "$cache_dir" ]; then
        cached=$(cached_path_of "$platform")
        if [ -f "$cached" ]; then
            actual=$(sha256_of "$cached")
            if [ "$actual" = "$want" ]; then
                cp "$cached" "$out"
                printf 'install_nextest: sha256 %s verified (tarball cache %s)\n' "$actual" "$cached"
                return 0
            fi
            annotate error "the tarball cache file $cached does not match the pinned sha256 of cargo-nextest $NEXTEST_VERSION ($platform); deleting it and downloading"
            rm -f "$cached"
        fi
    fi
    primary_status=0
    fetch_verified "$primary" "$out" "$want" || primary_status=$?
    if [ "$primary_status" -ne 0 ]; then
        if [ "$primary_status" -eq 2 ]; then
            annotate error "$primary served bytes that do not match the pinned sha256 of cargo-nextest $NEXTEST_VERSION ($platform); falling back to the GitHub release asset"
        else
            annotate warning "$primary did not deliver the pinned release; falling back to the GitHub release asset"
        fi
        fallback_status=0
        fetch_verified "$fallback" "$out" "$want" || fallback_status=$?
        if [ "$fallback_status" -eq 2 ]; then
            annotate error "$fallback served bytes that do not match the pinned sha256 of cargo-nextest $NEXTEST_VERSION ($platform)"
        fi
        if [ "$fallback_status" -ne 0 ]; then
            if [ "$primary_status" -eq 2 ] || [ "$fallback_status" -eq 2 ]; then
                die "no route delivered the pinned cargo-nextest $NEXTEST_VERSION ($platform) (errors above).
If NEXTEST_PINNED_VERSION was just bumped, the pinned checksums in this script
must be bumped with it (the header shows how to regenerate them). A checksum
mismatch on a route means the bytes it offered are not the release this repo
pinned: do not run them."
            fi
            die "no route delivered the pinned cargo-nextest $NEXTEST_VERSION ($platform): both downloads failed (see above)."
        fi
    fi
    if [ -n "$cache_dir" ] && ! store_in_cache "$out" "$platform"; then
        printf 'install_nextest: could not write the tarball cache %s\n' "$cache_dir" >&2
    fi
    return 0
}

# --fill-cache: every named platform's tarball verified into the cache, and
# nothing installed. A platform no source delivers ends the run nonzero. Each
# platform's first route is get.nexte.st; NEXTEST_PRIMARY_URL is read by the
# install only.
if [ "$mode" = fill ]; then
    [ -n "$cache_dir" ] || die "--fill-cache writes into CERULION_TOOL_TARBALLS, which is not set"
    for platform in "${fill_platforms[@]}"; do
        expected_sha_of "$platform" >/dev/null
    done
    require_sha256
    tmp=$(mktemp -d)
    trap 'rm -rf "$tmp"' EXIT
    for platform in "${fill_platforms[@]}"; do
        deliver "$platform" "$(primary_url_of "$NEXTEST_VERSION" "$platform")" "$tmp/$platform.tar.gz"
        cached=$(cached_path_of "$platform")
        if ! { [ -f "$cached" ] && [ "$(sha256_of "$cached")" = "$(expected_sha_of "$platform")" ]; }; then
            die "could not write $cached"
        fi
        printf 'install_nextest: %s holds the pinned %s tarball\n' "$cached" "$platform"
    done
    exit 0
fi

# The version as a WHOLE FIELD, never a prefix.
#
# `--version` prints `cargo-nextest 0.9.137 (…)`, so a substring test accepts
# 0.9.137 as a match for a pin of "0.9.13" — and because this check
# short-circuits before the download, pinning 0.9.13 on a machine that already
# had 0.9.137 would report "already on PATH" and never install the pinned
# build, silently defeating the pin this script exists to enforce.
installed_version() {
    command -v cargo-nextest >/dev/null 2>&1 || return 1
    cargo-nextest --version 2>/dev/null | head -1 | awk '{print $2}'
}

have_pinned() {
    [ "$(installed_version || true)" = "$NEXTEST_VERSION" ]
}

if have_pinned; then
    printf 'install_nextest: cargo-nextest %s already on PATH (%s)\n' \
        "$NEXTEST_VERSION" "$(command -v cargo-nextest)"
    exit 0
fi

# ARCH matters on Linux: `…/linux` is the x86_64 asset, so an aarch64 runner
# (`ubuntu-*-arm` is now offered, and this repo already cross-checks that
# target) would download a binary it cannot exec. macOS ships one universal
# binary, so it needs no arch arm.
case "$(uname -s)" in
    Linux)
        case "$(uname -m)" in
            x86_64|amd64)   platform=linux ;;
            aarch64|arm64)  platform=linux-arm ;;
            *) die "unsupported Linux arch '$(uname -m)' — add it here or install cargo-nextest by hand" ;;
        esac
        ;;
    Darwin) platform=mac ;;
    *)      die "unsupported OS '$(uname -s)' — add it here or install cargo-nextest by hand" ;;
esac

# `~/.cargo/bin` is on PATH for every job in this workflow (the toolchain
# action puts it there), so installing beside the other cargo binaries needs no
# PATH surgery and no `GITHUB_PATH` write.
dest="${CARGO_HOME:-$HOME/.cargo}/bin"
mkdir -p "$dest"
require_sha256

# The checksum is the pinned one above for every source; it is never read from
# a host or from the cache.
#
# NEXTEST_PRIMARY_URL replaces the first route. It changes where the bytes come
# from, not what runs, because whatever it serves must still match the pinned
# checksum. Pointing it at a closed port exercises the second route.
primary_url="${NEXTEST_PRIMARY_URL:-$(primary_url_of "$NEXTEST_VERSION" "$platform")}"

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
tarball="$tmp/nextest.tar.gz"

deliver "$platform" "$primary_url" "$tarball"

tar -xzf "$tarball" -C "$tmp" || die "the download was not a gzip tarball"
[ -f "$tmp/cargo-nextest" ] || die "the tarball did not contain a cargo-nextest binary"

chmod +x "$tmp/cargo-nextest"
mv "$tmp/cargo-nextest" "$dest/cargo-nextest"

# Verify the BINARY WE JUST WROTE, by path. Re-running `have_pinned` here would
# ask PATH instead, which conflates two different failures: a download that
# produced the wrong version, and a correct install into a directory PATH does
# not carry. The second is a real possibility whenever `CARGO_HOME` is set
# somewhere unusual, and reporting it as "wrong version" sends the reader
# hunting for a bad tarball.
# `--version` prints a multi-line report; the FIRST line carries the version.
# Compared as a WHOLE FIELD for the same reason `have_pinned` is — a trailing
# `*` glob would accept 0.9.137 for a pin of 0.9.13.
reported=$("$dest/cargo-nextest" --version 2>/dev/null | head -1 || true)
got=$(printf '%s' "$reported" | awk '{print $2}')
if [ -z "$reported" ]; then
    die "the installed binary at $dest/cargo-nextest does not run"
fi
if [ "$got" != "$NEXTEST_VERSION" ]; then
    die "the installed binary reports version '$got' ('$reported'), not $NEXTEST_VERSION"
fi
printf 'install_nextest: installed %s -> %s\n' "$reported" "$dest/cargo-nextest"

# PATH is a SEPARATE question, and a separate diagnostic. Everything downstream
# invokes `cargo nextest`, which resolves `cargo-nextest` through PATH.
if ! have_pinned; then
    die "installed $dest/cargo-nextest, but '$dest' is not on PATH (or an older
cargo-nextest shadows it) — \`cargo nextest\` would not find it. Add '$dest' to
PATH before this step."
fi
