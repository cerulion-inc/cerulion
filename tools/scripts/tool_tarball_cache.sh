#!/usr/bin/env bash
# tool_tarball_cache.sh: the actions/cache archive of pinned tool tarballs.
#
#   tools/scripts/tool_tarball_cache.sh key
#   CERULION_TOOL_TARBALLS=DIR tools/scripts/tool_tarball_cache.sh fill
#
# The archive is one directory holding the nextest and cargo-deny release
# tarballs of the platforms the CI jobs run on, each verified against the
# checksum pinned in its installer. A job that installs either tool restores the
# archive and points CERULION_TOOL_TARBALLS at it; the installer tests a cached
# tarball against the pin before it uses it, exactly as it tests a download,
# and downloads when the tarball is absent or fails that test.
#
# `key` prints `key=tool-tarballs-nextest-<version>-cargo-deny-<version>-<digest>`,
# the line form `$GITHUB_OUTPUT` reads. The digest is the SHA-256, cut to 16 hex
# digits, of both installers' pin lines, the two platform lists and CACHE_LAYOUT
# below, so the key changes whenever a pin, a list or the layout changes, and an
# archive restored under a key holds exactly the tarballs `fill` put in it.
#
# CACHE_LAYOUT names the file names the installers give cached tarballs (the
# release asset names, at the top of the directory). A change to those names
# changes CACHE_LAYOUT with it.
#
# `fill` runs each installer's `--fill-cache` over its platform list.
set -euo pipefail

NEXTEST_PLATFORMS=(linux mac)
CARGO_DENY_PLATFORMS=(linux)
CACHE_LAYOUT=release-asset-names-v1

script_dir=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)

die() {
    printf 'tool_tarball_cache: %s\n' "$1" >&2
    exit 1
}

# pins_of FILE PREFIX COUNT: the `PREFIX_PINNED_VERSION=` and `PREFIX_SHA256_*=`
# lines of FILE, refused unless there are exactly COUNT of them.
pins_of() {
    local file=$1 prefix=$2 count=$3 lines found
    lines=$(grep -E "^${prefix}_(PINNED_VERSION|SHA256_[A-Za-z0-9_]+)=" "$file" || true)
    found=$(printf '%s\n' "$lines" | grep -c . || true)
    [ "$found" -eq "$count" ] \
        || die "expected $count ${prefix}_ pin lines in $file, found $found"
    printf '%s\n' "$lines"
}

sha256_text() {
    if command -v sha256sum >/dev/null 2>&1; then
        sha256sum | cut -d' ' -f1
    else
        shasum -a 256 | cut -d' ' -f1
    fi
}

cmd_key() {
    local nextest_pins deny_pins nextest_version deny_version digest
    nextest_pins=$(pins_of "$script_dir/install_nextest.sh" NEXTEST 4)
    deny_pins=$(pins_of "$script_dir/install_cargo_deny.sh" CARGO_DENY 5)
    nextest_version=$(printf '%s\n' "$nextest_pins" | sed -n 's/^NEXTEST_PINNED_VERSION=//p')
    deny_version=$(printf '%s\n' "$deny_pins" | sed -n 's/^CARGO_DENY_PINNED_VERSION=//p')
    digest=$(printf '%s\n%s\nNEXTEST_PLATFORMS=%s\nCARGO_DENY_PLATFORMS=%s\nCACHE_LAYOUT=%s\n' \
        "$nextest_pins" "$deny_pins" "${NEXTEST_PLATFORMS[*]}" "${CARGO_DENY_PLATFORMS[*]}" \
        "$CACHE_LAYOUT" | sha256_text)
    printf 'key=tool-tarballs-nextest-%s-cargo-deny-%s-%s\n' "$nextest_version" "$deny_version" "${digest:0:16}"
}

cmd_fill() {
    [ -n "${CERULION_TOOL_TARBALLS:-}" ] || die "fill writes into CERULION_TOOL_TARBALLS, which is not set"
    "$script_dir/install_nextest.sh" --fill-cache "${NEXTEST_PLATFORMS[@]}"
    "$script_dir/install_cargo_deny.sh" --fill-cache "${CARGO_DENY_PLATFORMS[@]}"
}

case "${1-}:$#" in
    key:1)  cmd_key ;;
    fill:1) cmd_fill ;;
    *)      die "usage: tool_tarball_cache.sh key|fill" ;;
esac
