#!/usr/bin/env bash
# verify_published.sh: read back from crates.io what one version actually sent.
#
# `publish_preflight.sh` reads `cargo package --list`, the listing cargo would
# upload. This reads the other end: for every publishable workspace member at
# one version it fetches the sparse-index entry, the version metadata and the
# `.crate` archive the registry serves, and asserts these properties of each.
#
#   digest       the archive sha256 equals the index `cksum` AND the API
#                `checksum` (three surfaces, one value)
#   size         the archive byte length equals the API `crate_size`
#   paths        every archive member lies under `<name>-<version>/` and no
#                path carries a `..` segment; checked before extraction
#   agent-files  no `AGENTS.md` and no `CLAUDE.md` anywhere in the archive
#   test-tree    no member under a `tests/` directory at the crate root. A
#                `cfg(test)` module under `src/` is library source and ships
#                today (`src/state/tests.rs`, `src/test_sink.rs`,
#                `src/*_tests.rs`), so the rule matches the ROOT directory and
#                never a `*tests*` name
#   licence      the text files the packaged `license` expression requires are
#                in the archive, and that expression equals the API `license`
#   description  the packaged description carries no tracker id
#   readme       every relative link in the packaged README resolves to a file
#                inside the archive
#   provenance   `.cargo_vcs_info.json`'s git sha1 is the commit `v<version>`
#                points at; a checkout without that tag gets a warning
#
# The licence table below is the one `required_license_texts` matches on in
# crates/cerulion_hygiene/tests/crate_license_texts_test.rs:51-74; an expression
# neither knows is refused by both rather than passed with no text required.
#
# docs.rs is a WARNING line carrying the HTTP status. A 200 answers for the
# crate page whether or not a library was built, so it never fails a run.
#
# Each check prints one `PASS`, `FAIL` or `WARN` line per crate, then a count.
# Every crate is read before the exit status is decided, so one invocation names
# every defect it can see rather than the first.
#
# USAGE
#   verify_published.sh <version> [--crate <name>]... [--skip-docs]
#                       [--index-url <url>] [--wait-seconds <n>]
#
# Exit 0 when nothing failed, 1 when a check did, 2 on a usage error.
# Portable: bash 3.2+ (macOS), GNU and BSD userland. Needs cargo, curl, jq, tar
# and sha256sum or shasum.

set -euo pipefail

script_dir=$(CDPATH='' cd "$(dirname "$0")" && pwd)

INDEX_URL='https://index.crates.io'
API_URL='https://crates.io/api/v1/crates'
DOCS_URL='https://docs.rs/crate'
# crates.io answers the version metadata with HTTP 403 and an empty body to
# curl's default agent; the sparse index, the archive host and docs.rs do not
# ask. Every request here names itself, so a refusal cannot read as an outage.
USER_AGENT='cerulion-publish-verification (https://github.com/cerulion-inc/cerulion)'
# Spelled as a bracket class so this file does not carry the shape it refuses,
# the spelling tools/scripts/check_public_surface.py:27-29 uses on itself.
TRACKER_RE='[C]ER-[0-9]+'
# Both registry surfaces lag an accepted upload. The budget is wall clock from
# the start of the run and is shared by every crate, not spent again per crate.
WAIT_SECONDS=900
POLL_SECONDS=15

say() { printf 'verify_published: %s\n' "$*"; }
die() {
    printf 'verify_published: error: %s\n' "$*" >&2
    exit 2
}

usage() {
    printf '%s\n' \
        'usage: verify_published.sh <version> [--crate <name>]... [--skip-docs]' \
        '                           [--index-url <url>] [--wait-seconds <n>]' >&2
}

version=
skip_docs=0
wanted=
while [ "$#" -gt 0 ]; do
    case $1 in
        --crate)
            if [ "$#" -lt 2 ] || [ -z "$2" ]; then
                usage
                die "--crate needs a crate name"
            fi
            wanted="$wanted$2
"
            shift 2
            ;;
        --skip-docs)
            skip_docs=1
            shift
            ;;
        --index-url)
            [ "$#" -ge 2 ] || {
                usage
                die "--index-url needs a url"
            }
            INDEX_URL=${2%/}
            shift 2
            ;;
        --wait-seconds)
            [ "$#" -ge 2 ] || {
                usage
                die "--wait-seconds needs a count"
            }
            case $2 in
                '' | *[!0-9]*)
                    usage
                    die "--wait-seconds takes a whole number of seconds: $2"
                    ;;
            esac
            WAIT_SECONDS=$2
            shift 2
            ;;
        -h | --help)
            usage
            exit 0
            ;;
        -*)
            usage
            die "unknown argument: $1"
            ;;
        *)
            [ -z "$version" ] || {
                usage
                die "more than one version given: $version and $1"
            }
            version=$1
            shift
            ;;
    esac
done
[ -n "$version" ] || {
    usage
    die "a version is required"
}
printf '%s' "$version" |
    grep -Eq '^[0-9]+\.[0-9]+\.[0-9]+(-[0-9A-Za-z.-]+)?(\+[0-9A-Za-z.-]+)?$' ||
    die "not a version: $version"

for tool in cargo curl jq tar; do
    command -v "$tool" >/dev/null 2>&1 || die "$tool is required"
done
if command -v sha256sum >/dev/null 2>&1; then
    have_sha256sum=1
elif command -v shasum >/dev/null 2>&1; then
    have_sha256sum=0
else
    die "sha256sum or shasum is required"
fi

workdir=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-verify-published.XXXXXX")
trap 'rm -rf "$workdir"' EXIT

pass_count=0
fail_count=0
warn_count=0

# report <PASS|FAIL|WARN> <crate> <check> <detail>
report() {
    case $1 in
        PASS) pass_count=$((pass_count + 1)) ;;
        FAIL) fail_count=$((fail_count + 1)) ;;
        WARN) warn_count=$((warn_count + 1)) ;;
    esac
    printf 'verify_published: %s %s %s %s: %s\n' "$1" "$2" "$version" "$3" "$4"
}

sha256_of() {
    if [ "$have_sha256sum" -eq 1 ]; then
        sha256sum "$1"
    else
        shasum -a 256 "$1"
    fi | awk '{ print $1 }'
}

byte_size_of() {
    wc -c <"$1" | tr -d '[:space:]'
}

# The texts a `license` expression obliges the archive to carry, one per line.
# Exit 1 for an expression with no row: an archive whose licence nothing knows
# is refused, never passed with an empty list.
licence_texts() {
    case $1 in
        'AGPL-3.0-only') printf '%s\n' LICENSE ;;
        'MIT OR Apache-2.0') printf '%s\n' LICENSE-MIT LICENSE-APACHE ;;
        *) return 1 ;;
    esac
}

# index_path <lowercase name>: the sparse-index layout publish_crates.sh walks
# (1/a, 2/ab, 3/a/abc, ab/cd/abcdef).
index_path() {
    case ${#1} in
        1) printf '1/%s' "$1" ;;
        2) printf '2/%s' "$1" ;;
        3) printf '3/%s/%s' "${1:0:1}" "$1" ;;
        *) printf '%s/%s/%s' "${1:0:2}" "${1:2:2}" "$1" ;;
    esac
}

fetch() {
    curl -fsSL --user-agent "$USER_AGENT" --retry 3 --retry-delay 5 \
        --max-time 120 -o "$2" "$1"
}

# Set per crate by verify_crate, read by the two probes below.
probe_index_url=
probe_api_url=

probe_index() {
    fetch "$probe_index_url" "$workdir/index-body" >/dev/null 2>&1 || return 1
    jq -c --arg v "$version" 'select(.vers == $v)' <"$workdir/index-body" \
        >"$workdir/index-entry" 2>/dev/null || return 1
    [ -s "$workdir/index-entry" ]
}

probe_api() {
    fetch "$probe_api_url" "$workdir/api-body" >/dev/null 2>&1 || return 1
    jq -ce 'if (.version | type) == "object" then .version else empty end' \
        <"$workdir/api-body" >"$workdir/api-version" 2>/dev/null || return 1
    [ -s "$workdir/api-version" ]
}

# wait_for <deadline epoch> <probe>: run the probe until it succeeds or the
# deadline passes. No sleep reaches past the deadline.
wait_for() {
    local deadline=$1 probe=$2 now remaining nap
    while :; do
        if "$probe"; then
            return 0
        fi
        now=$(date +%s)
        [ "$now" -lt "$deadline" ] || return 1
        remaining=$((deadline - now))
        nap=$POLL_SECONDS
        [ "$nap" -lt "$remaining" ] || nap=$remaining
        sleep "$nap"
    done
}

# docs_status <name>: the HTTP status of the crate page, or `unreachable`.
docs_status() {
    local code
    code=$(curl -sS -L -o /dev/null -w '%{http_code}' --user-agent "$USER_AGENT" \
        --max-time 60 "$DOCS_URL/$1/$version" 2>/dev/null) || code=unreachable
    printf '%s' "$code"
}

# readme_targets <readme>: the target of every relative markdown link, one per
# line. Absolute schemes, root-relative paths and bare anchors are dropped; an
# `#anchor` and a quoted title after the target are cut.
readme_targets() {
    grep -oE '\]\([^)]+\)' "$1" |
        sed -e 's/^](//' -e 's/)$//' -e 's/[[:space:]].*$//' -e 's/#.*$//' |
        grep -vE '^(https?:|mailto:|ftp:|data:|/|$)' || true
}

# manifest_string <key> <manifest>: the value of a top-level single-line string
# key. Cargo writes the packaged manifest itself with every workspace
# inheritance already resolved, so these keys are present and flat.
manifest_string() {
    sed -n "s/^$1 *= *\"\(.*\)\"[[:space:]]*\$/\1/p" "$2" | head -n 1
}

# verify_crate <name>: every check for one crate. It returns 0 always; a failing
# check is counted by `report`, so one run reaches every crate.
verify_crate() {
    local name=$1
    local lower crate_dir archive entry meta deadline root prefix
    local index_cksum api_checksum api_size api_license
    local got_digest got_size listing rel base
    local declared want_text missing targets target tag want_sha got_sha code
    local bad_path bad_agent bad_test

    crate_dir="$workdir/crate"
    rm -rf "$crate_dir"
    mkdir -p "$crate_dir/tree"
    archive="$crate_dir/archive.crate"
    prefix="$name-$version/"

    lower=$(printf '%s' "$name" | tr '[:upper:]' '[:lower:]')
    probe_index_url="$INDEX_URL/$(index_path "$lower")"
    probe_api_url="$API_URL/$name/$version"
    deadline=$((run_start + WAIT_SECONDS))

    if ! wait_for "$deadline" probe_index; then
        report FAIL "$name" index "the sparse index did not carry this version within ${WAIT_SECONDS}s"
        return 0
    fi
    entry=$(cat "$workdir/index-entry")
    report PASS "$name" index "the sparse index carries this version"

    if ! wait_for "$deadline" probe_api; then
        report FAIL "$name" metadata "the version metadata did not answer within ${WAIT_SECONDS}s"
        return 0
    fi
    meta=$(cat "$workdir/api-version")
    report PASS "$name" metadata "the version metadata answers"

    index_cksum=$(printf '%s' "$entry" | jq -r '.cksum // ""')
    api_checksum=$(printf '%s' "$meta" | jq -r '.checksum // ""')
    api_size=$(printf '%s' "$meta" | jq -r '.crate_size // ""')
    api_license=$(printf '%s' "$meta" | jq -r '.license // ""')

    if ! fetch "$API_URL/$name/$version/download" "$archive"; then
        report FAIL "$name" download "the archive did not download"
        return 0
    fi
    got_digest=$(sha256_of "$archive")
    got_size=$(byte_size_of "$archive")
    report PASS "$name" download "$got_size bytes fetched from the registry"

    if [ -z "$index_cksum" ] || [ -z "$api_checksum" ]; then
        report FAIL "$name" digest "a registry surface named no checksum (index '$index_cksum', metadata '$api_checksum')"
    elif [ "$index_cksum" != "$api_checksum" ]; then
        report FAIL "$name" digest "the index says $index_cksum and the metadata says $api_checksum"
    elif [ "$got_digest" != "$index_cksum" ]; then
        report FAIL "$name" digest "the archive is $got_digest and both registry surfaces say $index_cksum"
    else
        report PASS "$name" digest "$got_digest on the archive, the index and the metadata"
    fi

    if [ -z "$api_size" ]; then
        report FAIL "$name" size "the metadata named no crate_size"
    elif [ "$got_size" != "$api_size" ]; then
        report FAIL "$name" size "the archive is $got_size bytes and the metadata says $api_size"
    else
        report PASS "$name" size "$got_size bytes, as the metadata says"
    fi

    if ! listing=$(tar -tzf "$archive" 2>/dev/null); then
        report FAIL "$name" paths "the archive is not a readable gzip tar"
        return 0
    fi
    bad_path=
    bad_agent=
    bad_test=
    while IFS= read -r rel; do
        [ -n "$rel" ] || continue
        case $rel in
            "$prefix"*) ;;
            *)
                [ -n "$bad_path" ] || bad_path=$rel
                continue
                ;;
        esac
        rel=${rel#"$prefix"}
        case "/$rel" in
            */../*) [ -n "$bad_path" ] || bad_path="$prefix$rel" ;;
        esac
        # The directory entries carry no content; the checks below name the file
        # that ships rather than the directory it sits in.
        case $rel in
            '' | */) continue ;;
        esac
        base=${rel##*/}
        if [ "$base" = AGENTS.md ] || [ "$base" = CLAUDE.md ]; then
            [ -n "$bad_agent" ] || bad_agent=$rel
        fi
        case $rel in
            tests/*) [ -n "$bad_test" ] || bad_test=$rel ;;
        esac
    done <<<"$listing"

    if [ -n "$bad_path" ]; then
        report FAIL "$name" paths "'$bad_path' is not a plain member of $prefix"
        return 0
    fi
    report PASS "$name" paths "every member lies under $prefix with no '..' segment"

    if [ -n "$bad_agent" ]; then
        report FAIL "$name" agent-files "the archive carries '$bad_agent'"
    else
        report PASS "$name" agent-files "no AGENTS.md and no CLAUDE.md"
    fi

    if [ -n "$bad_test" ]; then
        report FAIL "$name" test-tree "the archive carries '$bad_test' from the root tests directory"
    else
        report PASS "$name" test-tree "no member under a root tests directory"
    fi

    if ! tar -xzf "$archive" -C "$crate_dir/tree" 2>/dev/null; then
        report FAIL "$name" extract "the archive did not extract"
        return 0
    fi
    root="$crate_dir/tree/$prefix"
    if [ ! -f "$root/Cargo.toml" ]; then
        report FAIL "$name" extract "the archive carries no ${prefix}Cargo.toml"
        return 0
    fi
    report PASS "$name" extract "the archive extracted"

    declared=$(manifest_string license "$root/Cargo.toml")
    if [ -z "$declared" ]; then
        report FAIL "$name" licence "the packaged manifest declares no license"
    elif ! want_text=$(licence_texts "$declared"); then
        report FAIL "$name" licence "no licence row knows '$declared'; add it here and in crates/cerulion_hygiene/tests/crate_license_texts_test.rs"
    else
        missing=
        while IFS= read -r target; do
            [ -n "$target" ] || continue
            [ -f "$root/$target" ] || missing="$missing $target"
        done <<<"$want_text"
        if [ -n "$missing" ]; then
            report FAIL "$name" licence "'$declared' requires text the archive does not carry:$missing"
        else
            report PASS "$name" licence "'$declared' and every text it requires"
        fi
    fi

    if [ -z "$api_license" ]; then
        report FAIL "$name" licence-field "the metadata named no license"
    elif [ "$declared" != "$api_license" ]; then
        report FAIL "$name" licence-field "the packaged manifest says '$declared' and the metadata says '$api_license'"
    else
        report PASS "$name" licence-field "'$api_license' on the manifest and the metadata"
    fi

    if manifest_string description "$root/Cargo.toml" | grep -Eq "$TRACKER_RE"; then
        report FAIL "$name" description "the packaged description carries a tracker id"
    else
        report PASS "$name" description "no tracker id in the packaged description"
    fi

    if [ ! -f "$root/README.md" ]; then
        report WARN "$name" readme "the archive carries no README.md to read"
    else
        missing=
        targets=$(readme_targets "$root/README.md")
        while IFS= read -r target; do
            [ -n "$target" ] || continue
            [ -e "$root/$target" ] || missing="$missing $target"
        done <<<"$targets"
        if [ -n "$missing" ]; then
            report FAIL "$name" readme "the packaged README links to paths the archive does not carry:$missing"
        else
            report PASS "$name" readme "every relative README link resolves inside the archive"
        fi
    fi

    tag="v$version"
    got_sha=
    if [ -f "$root/.cargo_vcs_info.json" ]; then
        got_sha=$(jq -r '.git.sha1 // ""' <"$root/.cargo_vcs_info.json" 2>/dev/null) || got_sha=
    fi
    if ! command -v git >/dev/null 2>&1; then
        report WARN "$name" provenance "git is not runnable here, so $tag was not read"
    elif ! want_sha=$(git -C "$script_dir" rev-parse -q --verify "refs/tags/$tag^{commit}" 2>/dev/null); then
        report WARN "$name" provenance "this checkout carries no $tag, so the packaged commit was not compared"
    elif [ -z "$got_sha" ]; then
        report FAIL "$name" provenance "the archive carries no .cargo_vcs_info.json git sha1"
    elif [ "$got_sha" != "$want_sha" ]; then
        report FAIL "$name" provenance "the archive was packaged at $got_sha and $tag points at $want_sha"
    else
        report PASS "$name" provenance "packaged at $got_sha, the commit $tag points at"
    fi

    if [ "$skip_docs" -eq 1 ]; then
        return 0
    fi
    code=$(docs_status "$name")
    report WARN "$name" docs "the crate page answered $code; a page is not a built library"
    return 0
}

run_start=$(date +%s)

members=$(cargo metadata --no-deps --format-version 1 |
    jq -r '.packages[] | select(.publish != []) | .name')
[ -n "$members" ] || die "cargo metadata listed no publishable workspace member"

if [ -n "$wanted" ]; then
    while IFS= read -r name; do
        [ -n "$name" ] || continue
        printf '%s\n' "$members" | grep -Fxq "$name" ||
            die "$name is not a publishable workspace member"
    done <<<"$wanted"
    members=$(printf '%s' "$wanted" | grep -v '^$')
fi

total=$(printf '%s\n' "$members" | grep -c . || true)
say "verifying $total crate(s) at $version against $INDEX_URL and the crates.io api"
while IFS= read -r name; do
    [ -n "$name" ] || continue
    verify_crate "$name"
done <<<"$members"

say "$total crate(s) at $version: $pass_count PASS, $warn_count WARN, $fail_count FAIL"
if [ "$fail_count" -ne 0 ]; then
    say "FAILED"
    exit 1
fi
say "OK"
