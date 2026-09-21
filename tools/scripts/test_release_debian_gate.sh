#!/bin/sh

set -eu

# The fixture archives here test the version gate, not the package contents:
# they carry no rmw library, which build_deb.sh otherwise refuses.
CERULION_DEB_ALLOW_MISSING_RMW=1
export CERULION_DEB_ALLOW_MISSING_RMW

script_dir=$(CDPATH='' cd "$(dirname "$0")" && pwd)
workdir=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-release-debian-gate.XXXXXX")
cleanup() {
    rm -rf "$workdir"
}
trap cleanup EXIT

make_archive() {
    version=$1
    root="$workdir/cerulion-${version}-x86_64-unknown-linux-gnu"
    mkdir -p "$root"
    for binary in cerulion cerulion-netd cerulion-connectd; do
        printf '%s\n' '#!/bin/sh' > "$root/$binary"
        chmod 0755 "$root/$binary"
    done
    # build_deb.sh requires every license notice the archive ships, so a
    # fixture that carries only LICENSE is refused for a reason that has
    # nothing to do with the version gate these cases exercise.
    printf '%s\n' 'license' > "$root/LICENSE"
    printf '%s\n' 'notice' > "$root/NOTICE"
    printf '%s\n' 'bsd' > "$root/LICENSE-BSD-3-CLAUSE"
    printf '%s\n' 'third-party' > "$root/THIRD-PARTY-LICENSES.md"
    tar -czf "$workdir/cerulion-${version}-x86_64-unknown-linux-gnu.tar.gz" \
        -C "$workdir" "cerulion-${version}-x86_64-unknown-linux-gnu"
}

hyphenated_version=1.2.3-rc-1
make_archive "$hyphenated_version"
hyphenated_debs="$workdir/hyphenated-debs"
if "$script_dir/debian_version.sh" "$hyphenated_version" >/dev/null 2>"$workdir/hyphenated-error"; then
    printf '%s\n' 'error: unsupported hyphenated prerelease was accepted' >&2
    exit 1
fi
printf 'Debian package skipped for version %s: %s\n' "$hyphenated_version" \
    "$(cat "$workdir/hyphenated-error")"
grep -Fq "prerelease identifiers containing '-' are not supported" \
    "$workdir/hyphenated-error" || {
    printf '%s\n' 'error: invalid prerelease had the wrong version-validation diagnostic' >&2
    cat "$workdir/hyphenated-error" >&2
    exit 1
}
if "$script_dir/build_deb.sh" \
    "$workdir/cerulion-${hyphenated_version}-x86_64-unknown-linux-gnu.tar.gz" \
    amd64 "$hyphenated_debs" >"$workdir/hyphenated-build-output" \
    2>"$workdir/hyphenated-build-error"; then
    printf '%s\n' 'error: unsupported hyphenated prerelease produced a Debian package' >&2
    exit 1
fi
printf 'Debian package build rejected for version %s: %s\n' "$hyphenated_version" \
    "$(cat "$workdir/hyphenated-build-error")"
if find "$hyphenated_debs" -type f -name '*.deb' -print -quit 2>/dev/null |
    grep -q .; then
    printf '%s\n' 'error: rejected hyphenated prerelease left a Debian package behind' >&2
    exit 1
fi
printf '%s\n' 'hyphenated prerelease release has no Debian package'

for invalid_build_version in 1.2.3+build- 1.2.3+-build 1.2.3+build-a-b; do
    make_archive "$invalid_build_version"
    if "$script_dir/build_deb.sh" \
        "$workdir/cerulion-${invalid_build_version}-x86_64-unknown-linux-gnu.tar.gz" \
        amd64 "$workdir/invalid-build-debs" \
        >"$workdir/invalid-build-output" 2>"$workdir/invalid-build-error"; then
        printf 'error: invalid build identifier was accepted: %s\n' \
            "$invalid_build_version" >&2
        exit 1
    fi
    grep -Fq "build metadata identifiers containing '-'" \
        "$workdir/invalid-build-error" || {
        printf 'error: invalid build identifier had the wrong diagnostic: %s\n' \
            "$invalid_build_version" >&2
        cat "$workdir/invalid-build-error" >&2
        exit 1
    }
done
printf '%s\n' 'invalid build identifier release has no Debian package'

valid_version=1.2.3-rc.1
make_archive "$valid_version"
valid_debs="$workdir/valid-debs"
debian_version=$("$script_dir/debian_version.sh" "$valid_version")
"$script_dir/build_deb.sh" \
    "$workdir/cerulion-${valid_version}-x86_64-unknown-linux-gnu.tar.gz" \
    amd64 "$valid_debs" >/dev/null
test -f "$valid_debs/cerulion_${debian_version}_amd64.deb"
printf '%s\n' 'valid prerelease release has a Debian package'

citation_file="$workdir/CITATION.cff"
citation_gate="$script_dir/check_citation_release.sh"

run_citation_case() {
    name=$1
    expected_status=$2
    cff_date=$3
    cff_version=${4:-0.1.0}
    expected_version=${5:-0.1.0}
    tag_date=${6:-2026-09-07}
    run_date=${7:-2026-09-09}
    date_field=${8:-present}
    expected_substring=${9:-}
    cat > "$citation_file" <<EOF
version: "$cff_version"
EOF
    if [ "$date_field" = present ]; then
        cat >> "$citation_file" <<EOF
date-released: "$cff_date"
EOF
    fi
    if output=$(
        CITATION_TAG_DATE="$tag_date" \
        CITATION_RUN_DATE="$run_date" \
            "$citation_gate" "$expected_version" v0.1.0 "$citation_file" 2>&1
    ); then
        status=0
    else
        status=$?
    fi
    if [ "$status" -ne "$expected_status" ]; then
        printf 'error: citation case %s returned %s, expected %s\n%s\n' \
            "$name" "$status" "$expected_status" "$output" >&2
        exit 1
    fi
    if [ -n "$expected_substring" ] &&
        ! printf '%s\n' "$output" | grep -Fq -- "$expected_substring"; then
        printf 'error: citation case %s output did not contain %s\n%s\n' \
            "$name" "$expected_substring" "$output" >&2
        exit 1
    fi
    printf 'citation case %s: exit %s\n%s\n' "$name" "$status" "$output"
}

run_citation_case 'before-tag-date' 1 2026-09-06 0.1.0 0.1.0 2026-09-07 2026-09-09 present 'predates the tagged commit'
run_citation_case 'after-run-date' 1 2026-09-10 0.1.0 0.1.0 2026-09-07 2026-09-09 present 'is in the future'
run_citation_case 'equal-tag-date' 0 2026-09-07
run_citation_case 'equal-run-date' 0 2026-09-09
run_citation_case 'between-tag-and-run' 0 2026-09-08
run_citation_case 'malformed-date' 1 2026-9-9 0.1.0 0.1.0 2026-09-01 2027-01-01 present 'is malformed'
run_citation_case 'invalid-calendar-february' 1 2026-02-30 0.1.0 0.1.0 2026-02-01 2026-03-01 present 'is not a real calendar date'
run_citation_case 'invalid-calendar-month' 1 2026-13-01 0.1.0 0.1.0 2026-12-01 2027-01-15 present 'is not a real calendar date'
run_citation_case 'invalid-calendar-zero-month' 1 2026-00-10 0.1.0 0.1.0 2025-12-01 2026-01-15 present 'is not a real calendar date'
run_citation_case 'valid-leap-day' 0 2024-02-29 0.1.0 0.1.0 2024-02-01 2024-03-01
run_citation_case 'missing-date' 1 '' 0.1.0 0.1.0 2026-09-07 2026-09-09 absent 'date-released is missing'
run_citation_case 'empty-date' 1 '' 0.1.0 0.1.0 2026-09-07 2026-09-09 present 'date-released is missing'
run_citation_case 'version-mismatch' 1 2026-09-08 0.2.0 0.1.0 2026-09-07 2026-09-09 present 'does not match workspace version'
