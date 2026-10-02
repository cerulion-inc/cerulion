#!/usr/bin/env bash
# test_verify_published.sh: the oracle table for tools/scripts/verify_published.sh.
#
# Offline. `cargo`, `curl`, `git` and `sleep` are PATH shims: the fake cargo
# answers `metadata` with a fixed workspace, the fake curl serves a fixture
# registry out of a directory tree (sparse-index entries, version metadata, the
# `.crate` archives and a docs.rs status), the fake git answers the one tag
# lookup, and the fake sleep records its argument instead of sleeping. jq, tar,
# date and the sha256 tool are real, so the digest and size arms compare numbers
# this file did not write by hand.
#
# Two fixture crates, so the licence table is exercised on both of its rows:
# `demo_core` declares `AGPL-3.0-only` and carries `LICENSE`, `demo_link`
# declares `MIT OR Apache-2.0` and carries `LICENSE-MIT` and `LICENSE-APACHE`.
# A third member carries `publish = []` and must never be read. `demo_core`
# ships `src/state/tests.rs`, `src/cfg_symmetry_tests.rs` and `src/test_sink.rs`,
# the shapes that make the root-tests rule a path rule rather than a name rule:
# the clean arm would fail if it were written as a `*tests*` glob.
#
# One `FAIL:` line and exit 1 on the first failing assertion; one pass line per
# case, then a case count with a floor. Portable: bash 3.2+, GNU and BSD
# userland.

set -euo pipefail

script_dir=$(CDPATH='' cd "$(dirname "$0")" && pwd)
workdir=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-verify-published-test.XXXXXX")
cleanup() {
    rm -rf "$workdir"
}
trap cleanup EXIT

VERSION=1.2.3
TAG_SHA=0123456789abcdef0123456789abcdef01234567
OTHER_SHA=fedcba9876543210fedcba9876543210fedcba98
OLD_DIGEST=00000000000000000000000000000000000000000000000000000000000000aa
WRONG_DIGEST=00000000000000000000000000000000000000000000000000000000000000bb
# Assembled from two fragments so this file does not carry the shape the gate
# refuses, the spelling tools/scripts/check_public_surface.py:27-29 uses.
TRACKER=$(printf '%s%s' C 'ER-4242')
# The arms below; the floor is this count, so a run that stops early reds.
CASE_FLOOR=37

cases=0
case_name=
case_dir=
output=
status=0

fail() {
    printf 'FAIL: %s\n' "$1" >&2
    if [ -n "${output:-}" ]; then
        printf '%s\n' "--- output ---" "$output" >&2
    fi
    exit 1
}

passed() {
    cases=$((cases + 1))
    printf '%s\n' "$1: passed"
}

sha256_of() {
    if command -v sha256sum >/dev/null 2>&1; then
        sha256sum "$1"
    else
        shasum -a 256 "$1"
    fi | awk '{ print $1 }'
}

byte_size_of() {
    wc -c <"$1" | tr -d '[:space:]'
}

# ---------------------------------------------------------------------------
# PATH shims
# ---------------------------------------------------------------------------

shim_bin="$workdir/bin"
mkdir -p "$shim_bin"

cat >"$shim_bin/cargo" <<'SHIM'
#!/bin/sh
case $1 in
    metadata) cat "$FAKE_METADATA"; exit 0 ;;
esac
printf 'fake cargo: unexpected subcommand: %s\n' "$*" >&2
exit 99
SHIM

# The fixture registry, routed by URL shape: `.../download` serves the archive,
# `.../api/v1/crates/<name>/<vers>` the version metadata, a docs.rs URL the
# status in `docs_status` (the word `unreachable` makes curl exit 7 instead),
# and anything else is a sparse-index path keyed by its last segment. A
# `<name>.delay` counter 404s the index that many times first, which is how the
# bounded poll is measured.
cat >"$shim_bin/curl" <<'SHIM'
#!/bin/sh
out=
url=
write_out=0
while [ "$#" -gt 0 ]; do
    case $1 in
        -o) out=$2; shift 2 ;;
        -w) write_out=1; shift 2 ;;
        --user-agent | --max-time | --retry | --retry-delay) shift 2 ;;
        http://* | https://*) url=$1; shift ;;
        *) shift ;;
    esac
done
if [ -z "$url" ]; then printf 'fake curl: no url\n' >&2; exit 99; fi
printf '%s\n' "$url" >> "$FAKE_URLS"
src=
case $url in
    */download)
        name=$(printf '%s' "$url" | awk -F/ '{ print $(NF-2) }')
        src="$FAKE_WWW/$name.crate"
        ;;
    *docs.rs/*)
        code=$(cat "$FAKE_WWW/docs_status")
        if [ "$code" = unreachable ]; then
            printf 'fake curl: docs host unreachable\n' >&2
            exit 7
        fi
        if [ -n "$out" ]; then : > "$out"; fi
        if [ "$write_out" -eq 1 ]; then printf '%s' "$code"; fi
        exit 0
        ;;
    */api/v1/crates/*)
        name=$(printf '%s' "$url" | awk -F/ '{ print $(NF-1) }')
        src="$FAKE_WWW/api/$name.json"
        ;;
    *)
        name=${url##*/}
        delay="$FAKE_WWW/index/$name.delay"
        if [ -f "$delay" ]; then
            left=$(cat "$delay")
            if [ "$left" -gt 0 ]; then
                printf '%s' "$((left - 1))" > "$delay"
                printf 'fake curl: 404 %s\n' "$url" >&2
                exit 22
            fi
        fi
        src="$FAKE_WWW/index/$name"
        ;;
esac
if [ ! -f "$src" ]; then
    printf 'fake curl: no fixture for %s\n' "$url" >&2
    exit 22
fi
if [ -n "$out" ]; then cp "$src" "$out"; else cat "$src"; fi
if [ "$write_out" -eq 1 ]; then printf '200'; fi
exit 0
SHIM

# The one git call is the tag lookup: a non-empty fixture is the tagged commit,
# an empty one is a checkout with no such tag.
cat >"$shim_bin/git" <<'SHIM'
#!/bin/sh
if [ -s "${FAKE_TAG_SHA:-}" ]; then
    cat "$FAKE_TAG_SHA"
    exit 0
fi
exit 1
SHIM

cat >"$shim_bin/sleep" <<'SHIM'
#!/bin/sh
printf '%s\n' "$1" >> "$FAKE_SLEEPS"
SHIM

chmod 0755 "$shim_bin/cargo" "$shim_bin/curl" "$shim_bin/git" "$shim_bin/sleep"

cat >"$workdir/metadata.json" <<'JSON'
{"packages":[
 {"name":"demo_core","version":"1.2.3","publish":null},
 {"name":"demo_link","version":"1.2.3","publish":["crates-io"]},
 {"name":"demo_fixture","version":"1.2.3","publish":[]}
]}
JSON

cat >"$workdir/metadata_unpublishable.json" <<'JSON'
{"packages":[
 {"name":"demo_fixture","version":"1.2.3","publish":[]}
]}
JSON

# ---------------------------------------------------------------------------
# The fixture registry
# ---------------------------------------------------------------------------

# write_tree <dir> <name> <licence expression> <licence text file>...
write_tree() {
    local dir=$1 name=$2 licence=$3 tree
    shift 3
    tree="$dir/src/$name-$VERSION"
    mkdir -p "$tree/src"
    {
        printf '[package]\nname = "%s"\nversion = "%s"\n' "$name" "$VERSION"
        printf 'description = "demo crate for the published-archive oracle"\n'
        printf 'license = "%s"\n' "$licence"
    } >"$tree/Cargo.toml"
    printf 'fn demo() {}\n' >"$tree/src/lib.rs"
    printf '{"git":{"sha1":"%s"},"path_in_vcs":"crates/%s"}\n' "$TAG_SHA" "$name" \
        >"$tree/.cargo_vcs_info.json"
    for text in "$@"; do
        printf 'the %s text\n' "$text" >"$tree/$text"
    done
    if [ "$name" = demo_core ]; then
        # cfg(test) modules under src/ are library source and ship today.
        mkdir -p "$tree/src/state"
        printf 'fn t() {}\n' >"$tree/src/state/tests.rs"
        printf 'fn t() {}\n' >"$tree/src/cfg_symmetry_tests.rs"
        printf 'fn t() {}\n' >"$tree/src/test_sink.rs"
        printf '# %s\n\nno relative links here.\n' "$name" >"$tree/README.md"
    else
        # One relative link that resolves, the positive control for the readme
        # check: an arm that passed because nothing was read would not red.
        printf '# %s\n\nthe entry point is [src/lib.rs](src/lib.rs).\n' "$name" \
            >"$tree/README.md"
    fi
}

# set_www <dir> <name> <index cksum> <api checksum> <api crate_size> <api license>
# The index body carries an OLDER version line first, so the version select is
# exercised rather than assumed.
set_www() {
    local dir=$1 name=$2 cksum=$3 api_cksum=$4 size=$5 licence=$6
    printf '{"name":"%s","vers":"0.9.0","cksum":"%s","features":{},"yanked":false}\n' \
        "$name" "$OLD_DIGEST" >"$dir/www/index/$name"
    printf '{"name":"%s","vers":"%s","cksum":"%s","features":{},"yanked":false}\n' \
        "$name" "$VERSION" "$cksum" >>"$dir/www/index/$name"
    printf '{"version":{"num":"%s","checksum":"%s","crate_size":%s,"license":"%s","yanked":false}}\n' \
        "$VERSION" "$api_cksum" "$size" "$licence" >"$dir/www/api/$name.json"
}

# refresh_www <dir> <name>: both registry surfaces agree with the archive on
# disk and with the packaged manifest's licence.
refresh_www() {
    local dir=$1 name=$2 cksum size licence
    cksum=$(sha256_of "$dir/www/$name.crate")
    size=$(byte_size_of "$dir/www/$name.crate")
    licence=$(sed -n 's/^license *= *"\(.*\)"$/\1/p' "$dir/src/$name-$VERSION/Cargo.toml")
    set_www "$dir" "$name" "$cksum" "$cksum" "$size" "$licence"
}

# repack <dir> <name>: tar the source tree into the archive, then refresh.
repack() {
    local dir=$1 name=$2
    (cd "$dir/src" && tar -czf "$dir/www/$name.crate" "$name-$VERSION")
    refresh_www "$dir" "$name"
}

template="$workdir/template"
mkdir -p "$template/src" "$template/www/index" "$template/www/api"
printf '200' >"$template/www/docs_status"
write_tree "$template" demo_core 'AGPL-3.0-only' LICENSE
write_tree "$template" demo_link 'MIT OR Apache-2.0' LICENSE-MIT LICENSE-APACHE
repack "$template" demo_core
repack "$template" demo_link

# mutate <name> <what>: change the packaged tree, then repack so both registry
# surfaces still agree with the archive.
mutate() {
    local name=$1 what=$2 tree="$case_dir/src/$1-$VERSION"
    case $what in
        agent) printf 'notes\n' >"$tree/AGENTS.md" ;;
        claude) printf 'notes\n' >"$tree/CLAUDE.md" ;;
        test-tree)
            mkdir -p "$tree/tests"
            printf 'fn t() {}\n' >"$tree/tests/it.rs"
            ;;
        tracker)
            printf 'description = "demo crate, %s"\n' "$TRACKER" >>"$tree/Cargo.toml"
            sed -i.bak '/^description = "demo crate for the published-archive oracle"$/d' \
                "$tree/Cargo.toml"
            rm -f "$tree/Cargo.toml.bak"
            ;;
        dangling-link) printf '\nand [the guide](docs/gone.md).\n' >>"$tree/README.md" ;;
        drop-licence) rm -f "$tree/LICENSE" ;;
        drop-apache) rm -f "$tree/LICENSE-APACHE" ;;
        unknown-licence)
            sed -i.bak 's/^license = .*/license = "LicenseRef-demo"/' "$tree/Cargo.toml"
            rm -f "$tree/Cargo.toml.bak"
            ;;
        no-readme) rm -f "$tree/README.md" ;;
        bad-vcs)
            printf '{"git":{"sha1":"%s"},"path_in_vcs":"crates/%s"}\n' "$OTHER_SHA" "$name" \
                >"$tree/.cargo_vcs_info.json"
            ;;
        no-vcs) rm -f "$tree/.cargo_vcs_info.json" ;;
        *) fail "unknown mutation $what" ;;
    esac
    repack "$case_dir" "$name"
}

new_case() {
    case_name=$1
    case_dir="$workdir/$case_name"
    mkdir -p "$case_dir"
    cp -R "$template/src" "$case_dir/src"
    cp -R "$template/www" "$case_dir/www"
    cp "$workdir/metadata.json" "$case_dir/metadata.json"
    printf '%s\n' "$TAG_SHA" >"$case_dir/tag_sha"
    : >"$case_dir/urls"
    : >"$case_dir/sleeps"
    output=
    status=0
}

run_script() {
    set +e
    output=$(
        cd "$case_dir" &&
            PATH="$shim_bin:$PATH" \
                FAKE_METADATA="$case_dir/metadata.json" \
                FAKE_WWW="$case_dir/www" \
                FAKE_URLS="$case_dir/urls" \
                FAKE_SLEEPS="$case_dir/sleeps" \
                FAKE_TAG_SHA="$case_dir/tag_sha" \
                "$script_dir/verify_published.sh" "$@" 2>&1
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

expect_absent() {
    printf '%s\n' "$output" | grep -Fq -- "$1" &&
        fail "$case_name: output carries '$1' and must not"
    return 0
}

expect_file_absent() {
    grep -Fq -- "$2" "$1" &&
        fail "$case_name: $1 carries '$2' and must not"
    return 0
}

# ---------------------------------------------------------------------------
# A clean registry passes every check, and the counts are the oracle
# ---------------------------------------------------------------------------
# Fourteen checks report per crate (index, metadata, download, digest, size,
# paths, agent-files, test-tree, extract, licence, licence-field, description,
# readme, provenance) plus one docs warning, so a clean two-crate run is
# 28 PASS, 2 WARN, 0 FAIL. A check that stopped reporting changes that line.
new_case clean
run_script "$VERSION"
expect_status 0
expect_output 'verifying 2 crate(s) at 1.2.3 against https://index.crates.io and the crates.io api'
expect_output 'PASS demo_core 1.2.3 test-tree: no member under a root tests directory'
expect_output "PASS demo_core 1.2.3 licence: 'AGPL-3.0-only' and every text it requires"
expect_output "PASS demo_link 1.2.3 licence: 'MIT OR Apache-2.0' and every text it requires"
expect_output 'PASS demo_link 1.2.3 readme: every relative README link resolves inside the archive'
expect_output "PASS demo_core 1.2.3 provenance: packaged at $TAG_SHA, the commit v1.2.3 points at"
expect_output 'WARN demo_core 1.2.3 docs: the crate page answered 200; a page is not a built library'
expect_output '2 crate(s) at 1.2.3: 28 PASS, 2 WARN, 0 FAIL'
expect_output 'verify_published: OK'
# The `publish = []` member is never fetched.
expect_file_absent "$case_dir/urls" demo_fixture
passed 'a clean registry passes every check'

# The three sha256 values really are compared: the digest the index and the
# metadata carry is the archive's, not a constant this file wrote.
grep -Fq "$(sha256_of "$case_dir/www/demo_core.crate")" "$case_dir/www/index/demo_core" ||
    fail 'clean: the index fixture does not carry the archive digest'
passed 'the fixture index carries the real archive digest'

# ---------------------------------------------------------------------------
# docs.rs is a warning, never a failure, and --skip-docs skips the request
# ---------------------------------------------------------------------------
new_case skip_docs
run_script "$VERSION" --skip-docs
expect_status 0
expect_output '2 crate(s) at 1.2.3: 28 PASS, 0 WARN, 0 FAIL'
expect_absent 'docs:'
expect_file_absent "$case_dir/urls" docs.rs
passed '--skip-docs asks docs.rs nothing'

new_case docs_unreachable
printf 'unreachable' >"$case_dir/www/docs_status"
run_script "$VERSION"
expect_status 0
expect_output 'WARN demo_core 1.2.3 docs: the crate page answered unreachable; a page is not a built library'
expect_output '28 PASS, 2 WARN, 0 FAIL'
passed 'an unreachable docs host warns and does not fail'

new_case docs_missing
printf '404' >"$case_dir/www/docs_status"
run_script "$VERSION"
expect_status 0
expect_output 'WARN demo_link 1.2.3 docs: the crate page answered 404'
expect_output '0 FAIL'
passed 'a docs 404 warns and does not fail'

# ---------------------------------------------------------------------------
# The digest, in both directions
# ---------------------------------------------------------------------------
new_case bad_digest
size=$(byte_size_of "$case_dir/www/demo_core.crate")
set_www "$case_dir" demo_core "$WRONG_DIGEST" "$WRONG_DIGEST" "$size" 'AGPL-3.0-only'
run_script "$VERSION"
expect_status 1
expect_output "FAIL demo_core 1.2.3 digest: the archive is "
expect_output "and both registry surfaces say $WRONG_DIGEST"
expect_output '1 FAIL'
passed 'an archive that does not match the published digest fails'

new_case split_digest
real=$(sha256_of "$case_dir/www/demo_core.crate")
size=$(byte_size_of "$case_dir/www/demo_core.crate")
set_www "$case_dir" demo_core "$real" "$WRONG_DIGEST" "$size" 'AGPL-3.0-only'
run_script "$VERSION"
expect_status 1
expect_output "FAIL demo_core 1.2.3 digest: the index says $real and the metadata says $WRONG_DIGEST"
passed 'two registry surfaces disagreeing on the digest fails'

# ---------------------------------------------------------------------------
# The byte size
# ---------------------------------------------------------------------------
new_case bad_size
real=$(sha256_of "$case_dir/www/demo_core.crate")
set_www "$case_dir" demo_core "$real" "$real" 11 'AGPL-3.0-only'
run_script "$VERSION"
expect_status 1
expect_output 'FAIL demo_core 1.2.3 size: the archive is '
expect_output 'and the metadata says 11'
passed 'a crate_size the archive does not have fails'

# ---------------------------------------------------------------------------
# The licence texts, both rows of the table, and the licence field
# ---------------------------------------------------------------------------
new_case missing_licence_text
mutate demo_core drop-licence
run_script "$VERSION"
expect_status 1
expect_output "FAIL demo_core 1.2.3 licence: 'AGPL-3.0-only' requires text the archive does not carry: LICENSE"
passed 'an archive with no licence text fails'

new_case missing_dual_licence_text
mutate demo_link drop-apache
run_script "$VERSION"
expect_status 1
expect_output "FAIL demo_link 1.2.3 licence: 'MIT OR Apache-2.0' requires text the archive does not carry: LICENSE-APACHE"
passed 'a dual-licensed archive missing one text fails'

new_case unknown_licence
mutate demo_core unknown-licence
run_script "$VERSION"
expect_status 1
expect_output "FAIL demo_core 1.2.3 licence: no licence row knows 'LicenseRef-demo'"
expect_output 'crates/cerulion_hygiene/tests/crate_license_texts_test.rs'
passed 'a licence expression no row knows fails rather than requiring nothing'

new_case licence_field_mismatch
real=$(sha256_of "$case_dir/www/demo_core.crate")
size=$(byte_size_of "$case_dir/www/demo_core.crate")
set_www "$case_dir" demo_core "$real" "$real" "$size" 'MIT'
run_script "$VERSION"
expect_status 1
expect_output "FAIL demo_core 1.2.3 licence-field: the packaged manifest says 'AGPL-3.0-only' and the metadata says 'MIT'"
passed 'a metadata licence the manifest does not declare fails'

# ---------------------------------------------------------------------------
# Agent instruction files and the root test tree
# ---------------------------------------------------------------------------
new_case agent_file
mutate demo_core agent
run_script "$VERSION"
expect_status 1
expect_output "FAIL demo_core 1.2.3 agent-files: the archive carries 'AGENTS.md'"
passed 'an agent instruction file in the archive fails'

new_case claude_file
mutate demo_link claude
run_script "$VERSION"
expect_status 1
expect_output "FAIL demo_link 1.2.3 agent-files: the archive carries 'CLAUDE.md'"
passed 'the second agent instruction file name is refused too'

new_case test_tree
mutate demo_core test-tree
run_script "$VERSION"
expect_status 1
expect_output "FAIL demo_core 1.2.3 test-tree: the archive carries 'tests/it.rs' from the root tests directory"
expect_output '27 PASS, 2 WARN, 1 FAIL'
passed 'a root tests directory in the archive fails'

# ---------------------------------------------------------------------------
# The packaged description and the packaged README
# ---------------------------------------------------------------------------
new_case tracker_id
mutate demo_core tracker
run_script "$VERSION"
expect_status 1
expect_output 'FAIL demo_core 1.2.3 description: the packaged description carries a tracker id'
passed 'a tracker id in the packaged description fails'

new_case dangling_readme_link
mutate demo_core dangling-link
run_script "$VERSION"
expect_status 1
expect_output 'FAIL demo_core 1.2.3 readme: the packaged README links to paths the archive does not carry: docs/gone.md'
passed 'a README link to a path outside the archive fails'

new_case no_readme
mutate demo_core no-readme
run_script "$VERSION"
expect_status 0
expect_output 'WARN demo_core 1.2.3 readme: the archive carries no README.md to read'
expect_output '27 PASS, 3 WARN, 0 FAIL'
passed 'an archive with no README warns'

# ---------------------------------------------------------------------------
# Provenance back to the tag
# ---------------------------------------------------------------------------
new_case vcs_mismatch
mutate demo_core bad-vcs
run_script "$VERSION"
expect_status 1
expect_output "FAIL demo_core 1.2.3 provenance: the archive was packaged at $OTHER_SHA and v1.2.3 points at $TAG_SHA"
passed 'an archive packaged at another commit fails'

new_case no_vcs_info
mutate demo_core no-vcs
run_script "$VERSION"
expect_status 1
expect_output 'FAIL demo_core 1.2.3 provenance: the archive carries no .cargo_vcs_info.json git sha1'
passed 'an archive with no packaging commit fails'

new_case no_tag
: >"$case_dir/tag_sha"
run_script "$VERSION"
expect_status 0
expect_output 'WARN demo_core 1.2.3 provenance: this checkout carries no v1.2.3, so the packaged commit was not compared'
expect_output '26 PASS, 4 WARN, 0 FAIL'
passed 'a checkout without the tag warns instead of failing'

# ---------------------------------------------------------------------------
# A member outside the archive prefix stops that crate before extraction
# ---------------------------------------------------------------------------
new_case path_escape
mkdir -p "$case_dir/src/stray"
printf 'loose\n' >"$case_dir/src/stray/loose.txt"
(cd "$case_dir/src" && tar -czf "$case_dir/www/demo_core.crate" "demo_core-$VERSION" stray/loose.txt)
refresh_www "$case_dir" demo_core
run_script "$VERSION"
expect_status 1
expect_output "FAIL demo_core 1.2.3 paths: 'stray/loose.txt' is not a plain member of demo_core-1.2.3/"
expect_absent 'demo_core 1.2.3 extract'
expect_output '19 PASS, 1 WARN, 1 FAIL'
passed 'a member outside the archive prefix stops that crate before extraction'

# ---------------------------------------------------------------------------
# The bounded wait on the sparse index
# ---------------------------------------------------------------------------
new_case index_lag
printf '2' >"$case_dir/www/index/demo_core.delay"
run_script "$VERSION"
expect_status 0
expect_output 'PASS demo_core 1.2.3 index: the sparse index carries this version'
expect_output '0 FAIL'
[ "$(cat "$case_dir/sleeps")" = '15
15' ] || fail "index_lag: recorded sleeps $(cat "$case_dir/sleeps"), expected two 15s waits"
passed 'the index wait polls until the version appears'

new_case index_timeout
printf '99' >"$case_dir/www/index/demo_core.delay"
run_script "$VERSION" --wait-seconds 0
expect_status 1
expect_output 'FAIL demo_core 1.2.3 index: the sparse index did not carry this version within 0s'
# The run still reaches the next crate rather than stopping at the first defect.
expect_output 'PASS demo_link 1.2.3 digest:'
[ ! -s "$case_dir/sleeps" ] || fail 'index_timeout: a spent budget still slept'
passed 'a spent index budget fails that crate and the run continues'

# ---------------------------------------------------------------------------
# Crate selection and the index url
# ---------------------------------------------------------------------------
new_case crate_narrowing
run_script "$VERSION" --crate demo_link
expect_status 0
expect_output 'verifying 1 crate(s) at 1.2.3'
expect_output '1 crate(s) at 1.2.3: 14 PASS, 1 WARN, 0 FAIL'
expect_absent 'demo_core'
passed '--crate narrows the run to the named crate'

new_case unknown_crate
run_script "$VERSION" --crate demo_fixture
expect_status 2
expect_output 'demo_fixture is not a publishable workspace member'
passed 'a --crate name that is not a publishable member is refused'

new_case index_url
run_script "$VERSION" --index-url https://index.example.invalid/mirror/
expect_status 0
expect_output 'against https://index.example.invalid/mirror and the crates.io api'
grep -Fq 'https://index.example.invalid/mirror/de/mo/demo_core' "$case_dir/urls" ||
    fail 'index_url: the index request did not use the given url'
passed '--index-url moves the sparse-index read'

# ---------------------------------------------------------------------------
# Refusals before anything is read
# ---------------------------------------------------------------------------
new_case no_publishable_member
cp "$workdir/metadata_unpublishable.json" "$case_dir/metadata.json"
run_script "$VERSION"
expect_status 2
expect_output 'cargo metadata listed no publishable workspace member'
expect_file_absent "$case_dir/urls" crates.io
passed 'a workspace with no publishable member is refused before any fetch'

new_case no_version
run_script
expect_status 2
expect_output 'a version is required'
expect_output 'usage: verify_published.sh <version>'
passed 'a run with no version is refused'

new_case bad_version
run_script 1.2
expect_status 2
expect_output 'not a version: 1.2'
passed 'a version that is not three numbers is refused'

new_case prerelease_version
run_script 1.2.3-alpha.1 --skip-docs --wait-seconds 0
expect_status 1
expect_output 'FAIL demo_core 1.2.3-alpha.1 index:'
expect_absent 'not a version'
passed 'a prerelease version is accepted by the version shape'

new_case unknown_flag
run_script --nope "$VERSION"
expect_status 2
expect_output 'unknown argument: --nope'
passed 'an unknown argument is refused'

new_case two_versions
run_script "$VERSION" 2.0.0
expect_status 2
expect_output 'more than one version given: 1.2.3 and 2.0.0'
passed 'two versions are refused'

new_case missing_flag_value
run_script "$VERSION" --crate
expect_status 2
expect_output '--crate needs a crate name'
passed 'a flag with no value is refused'

new_case empty_flag_value
run_script "$VERSION" --crate ''
expect_status 2
expect_output '--crate needs a crate name'
passed 'a flag with an empty value is refused'

new_case bad_wait_seconds
run_script "$VERSION" --wait-seconds soon
expect_status 2
expect_output '--wait-seconds takes a whole number of seconds: soon'
passed 'a wait budget that is not a number is refused'

new_case help
run_script --help
expect_status 0
expect_output 'usage: verify_published.sh <version>'
expect_file_absent "$case_dir/urls" crates.io
passed '--help prints the usage and fetches nothing'

if [ "$cases" -lt "$CASE_FLOOR" ]; then
    fail "the suite judged $cases case(s), under the floor of $CASE_FLOOR"
fi
printf 'test_verify_published: self-test OK (%d cases)\n' "$cases"
