#!/usr/bin/env bash
# test_tool_tarball_cache.sh: offline self-test for tool_tarball_cache.sh.
#
#   tools/scripts/test_tool_tarball_cache.sh
#
# `key` runs over copies of the script and of both installers, so a pin line
# or a platform list can be changed in a copy and the key compared. `fill`
# runs over stub installers that log their arguments and the cache directory
# they were given.
#
# Exit 0 = every case held. Nonzero = the first failed case, named on stderr.
set -euo pipefail

script_dir=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)
work=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-tool-tarball-cache-test.XXXXXX")
trap 'rm -rf "$work"' EXIT

fail() {
    printf 'test_tool_tarball_cache: FAIL: %s\n' "$1" >&2
    exit 1
}

# copy NAME: a fresh directory holding copies of the three scripts.
copy() {
    local dir=$work/$1
    mkdir -p "$dir"
    cp "$script_dir/tool_tarball_cache.sh" "$script_dir/install_nextest.sh" \
        "$script_dir/install_cargo_deny.sh" "$dir/"
    printf '%s' "$dir"
}

key_of() { "$1/tool_tarball_cache.sh" key; }

nextest_version=$(sed -n 's/^NEXTEST_PINNED_VERSION=//p' "$script_dir/install_nextest.sh")
deny_version=$(sed -n 's/^CARGO_DENY_PINNED_VERSION=//p' "$script_dir/install_cargo_deny.sh")

base=$(copy base)
base_key=$(key_of "$base")
case "$base_key" in
    "key=tool-tarballs-nextest-$nextest_version-cargo-deny-$deny_version-"????????????????) ;;
    *) fail "the key line is '$base_key'" ;;
esac
printf '%s' "${base_key##*-}" | grep -qE '^[0-9a-f]{16}$' || fail "the digest of '$base_key' is not 16 hex digits"
[ "$(key_of "$base")" = "$base_key" ] || fail "two runs over the same pins gave different keys"
[ "$(key_of "$script_dir")" = "$base_key" ] || fail "the copy and the tree give different keys"
printf 'the key names both versions and a digest, and is stable: passed\n'

# Each pin line and each platform list moves the key.
expect_moved() {
    local name=$1 file=$2 expr=$3 dir
    dir=$(copy "$name")
    sed -e "$expr" "$dir/$file" > "$dir/$file.new" && mv "$dir/$file.new" "$dir/$file"
    chmod +x "$dir/$file"
    cmp -s "$dir/$file" "$script_dir/$file" && fail "$name: the edit changed nothing in $file"
    [ "$(key_of "$dir")" != "$base_key" ] || fail "$name: the key did not change"
}
expect_moved nextest-sha install_nextest.sh 's/^\(NEXTEST_SHA256_mac=\).*/\1aaaa/'
expect_moved nextest-version install_nextest.sh 's/^NEXTEST_PINNED_VERSION=.*/NEXTEST_PINNED_VERSION=9.9.9/'
expect_moved deny-sha install_cargo_deny.sh 's/^\(CARGO_DENY_SHA256_linux=\).*/\1bbbb/'
expect_moved deny-version install_cargo_deny.sh 's/^CARGO_DENY_PINNED_VERSION=.*/CARGO_DENY_PINNED_VERSION=9.9.9/'
expect_moved nextest-platforms tool_tarball_cache.sh 's/^NEXTEST_PLATFORMS=.*/NEXTEST_PLATFORMS=(linux)/'
expect_moved deny-platforms tool_tarball_cache.sh 's/^CARGO_DENY_PLATFORMS=.*/CARGO_DENY_PLATFORMS=(linux mac-arm)/'
expect_moved cache-layout tool_tarball_cache.sh 's/^CACHE_LAYOUT=.*/CACHE_LAYOUT=other/'
case "$(key_of "$work/nextest-version")" in
    key=tool-tarballs-nextest-9.9.9-*) ;;
    *) fail "a moved nextest version is not named in the key" ;;
esac
printf 'every pin line, platform list and the cache layout move the key: passed\n'

# A pin line the key cannot find is refused, not hashed as absent.
dir=$(copy missing-pin)
grep -v '^CARGO_DENY_SHA256_mac_arm=' "$dir/install_cargo_deny.sh" > "$dir/x" && mv "$dir/x" "$dir/install_cargo_deny.sh"
status=0
key_of "$dir" > "$work/missing-pin.out" 2>&1 || status=$?
[ "$status" -ne 0 ] || fail "a missing pin line still produced a key"
grep -q 'expected 5 CARGO_DENY_ pin lines' "$work/missing-pin.out" || fail "the missing pin line was not named"
printf 'a missing pin line is refused: passed\n'

# fill: each installer's --fill-cache over its platform list.
dir=$(copy fill)
for installer in install_nextest.sh install_cargo_deny.sh; do
    cat > "$dir/$installer" <<STUB
#!/bin/sh
printf '%s %s %s\n' "$installer" "\$CERULION_TOOL_TARBALLS" "\$*" >> "$work/fill.log"
exit "\${STUB_STATUS:-0}"
STUB
    chmod +x "$dir/$installer"
done
CERULION_TOOL_TARBALLS=/cache "$dir/tool_tarball_cache.sh" fill > /dev/null
[ "$(cat "$work/fill.log")" = "install_nextest.sh /cache --fill-cache linux mac
install_cargo_deny.sh /cache --fill-cache linux" ] || fail "fill ran: $(cat "$work/fill.log")"
: > "$work/fill.log"
status=0
CERULION_TOOL_TARBALLS=/cache STUB_STATUS=3 "$dir/tool_tarball_cache.sh" fill > /dev/null 2>&1 || status=$?
[ "$status" -ne 0 ] || fail "fill passed with a failing installer"
: > "$work/fill.log"
status=0
env -u CERULION_TOOL_TARBALLS "$dir/tool_tarball_cache.sh" fill > /dev/null 2>&1 || status=$?
[ "$status" -ne 0 ] || fail "fill ran without CERULION_TOOL_TARBALLS"
[ ! -s "$work/fill.log" ] || fail "fill ran an installer without CERULION_TOOL_TARBALLS"
status=0
"$dir/tool_tarball_cache.sh" bogus > /dev/null 2>&1 || status=$?
[ "$status" -ne 0 ] || fail "an unknown command was accepted"
printf 'fill runs each installer over its platform list: passed\n'

printf 'test_tool_tarball_cache: all cases passed\n'
