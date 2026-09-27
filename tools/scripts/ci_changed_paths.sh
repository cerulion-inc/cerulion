#!/usr/bin/env bash
# ci_changed_paths.sh — classify a pull request's changed paths for ci.yml.
#
# Reads one path per line on stdin and writes `<class>=true|false` lines on
# stdout, in the shape `$GITHUB_OUTPUT` wants:
#
#   git diff --name-only "$BASE" HEAD | tools/scripts/ci_changed_paths.sh >> "$GITHUB_OUTPUT"
#
# WHAT A CLASS MEANS, and the direction matters. A class here says "this change
# can reach that job", and ci.yml uses it to RUN a job it would otherwise skip.
# Nothing here skips a job that would otherwise run, so a rule that is too
# NARROW costs a run of a job on `main` instead of on the pull request, which is
# where the packaging work is caught today anyway, and a rule that is too WIDE
# costs runner minutes. Neither direction can make a pull request merge without
# a gate it has today.
#
# `packaging` — the inputs of the `deb-smoke` job (Debian and APT package
# smoke). That job is push-only because it is 22 minutes of compression-bound
# work that almost no pull request can break. The ones that CAN are the ones
# that touch the scripts it drives, the license inventory it assembles, or the
# packaging documentation that describes what it produces: for those, finding
# out on `main` means a revert instead of a red check.
#
# The list is derived from the job's own steps rather than guessed. Every entry
# is a file the job reads or a script it executes, DIRECTLY or through another
# script: `check_version_sync.sh` and `check_citation_release.sh` are in because
# `test_workspace_version.sh` and `test_release_debian_gate.sh` execute them, and
# the root `LICENSE` is in because the job stages it and `build_deb.sh` refuses
# an archive that does not carry it. `apt-repo.yml` is in because it publishes
# what this job smoke-tests.
#
# Two files the job does touch are deliberately OUT. `ci.yml`, because a workflow
# edit that does not touch packaging should not pay 22 minutes to learn that. And
# the root `README.md`, staged beside the license by the same step: it is edited
# far too often to put every documentation pull request behind a 22-minute job,
# and a pull request that deletes it and nothing else is still caught by `main`'s
# push run.
#
# Renames reach here as a DELETE plus an ADD, never as a destination path alone:
# the workflow diffs with `--no-renames` so that renaming a listed input away
# still matches on the name it had.
#
# Unreadable or empty input yields `false` for every class: this gate only ever
# ADDS work, so the quiet answer is the same behaviour the workflow had before.
#
# `--self-test` runs the table below and exits nonzero on the first miss.
set -euo pipefail

# One prefix or exact path per line. A prefix ends in `/`.
PACKAGING_PATHS='
tools/scripts/build_deb.sh
tools/scripts/test_build_deb.sh
tools/scripts/build_apt_repo.sh
tools/scripts/publish_apt_repo.sh
tools/scripts/build_keyring_deb.sh
tools/scripts/check_apt_keyring_coverage.sh
tools/scripts/test_apt_publication_order.sh
tools/scripts/test_release_debian_gate.sh
tools/scripts/check_citation_release.sh
tools/scripts/debian_version.sh
tools/scripts/workspace_version.sh
tools/scripts/test_workspace_version.sh
tools/scripts/check_version_sync.sh
tools/scripts/verify_rmw_deb.sh
tools/scripts/verify_rmw_deb_container.sh
tools/release/
docs/packaging/
docs/legal/
LICENSE
.github/workflows/apt-repo.yml
'

matches_packaging() {
    # $1 is one changed path.
    local path=$1 entry
    while IFS= read -r entry; do
        [ -n "$entry" ] || continue
        case "$entry" in
            */) case "$path" in "$entry"*) return 0 ;; esac ;;
            *)  [ "$path" = "$entry" ] && return 0 ;;
        esac
    done <<< "$PACKAGING_PATHS"
    return 1
}

classify() {
    local packaging=false path
    while IFS= read -r path; do
        [ -n "$path" ] || continue
        if matches_packaging "$path"; then
            packaging=true
        fi
    done
    printf 'packaging=%s\n' "$packaging"
}

self_test() {
    local fails=0
    # `<input paths>|<expected packaging>`; `;` separates paths.
    local cases='
tools/scripts/build_deb.sh|true
docs/packaging/apt.md|true
tools/release/about.toml|true
.github/workflows/apt-repo.yml|true
docs/legal/NOTICE|true
tools/scripts/check_version_sync.sh|true
tools/scripts/check_citation_release.sh|true
LICENSE|true
LICENSE-BSD-3-CLAUSE|false
crates/cerulion_core/src/wire.rs|false
.github/workflows/ci.yml|false
README.md|false
tools/scripts/install.sh|false
tools/scripts/build_deb.sh.orig|false
docs/packaging|false
crates/cerulion_core/src/wire.rs;tools/scripts/build_apt_repo.sh|true
crates/cerulion_core/src/wire.rs;README.md|false
|false
'
    local line paths want got
    while IFS= read -r line; do
        [ -n "$line" ] || continue
        paths=${line%|*}
        want=${line##*|}
        got=$(printf '%s\n' "${paths//;/$'\n'}" | classify)
        if [ "$got" != "packaging=$want" ]; then
            printf 'ci_changed_paths self-test: %s -> %s, wanted packaging=%s\n' \
                "${paths:-<empty>}" "$got" "$want" >&2
            fails=$((fails + 1))
        fi
    done <<< "$cases"
    if [ "$fails" -ne 0 ]; then
        printf 'ci_changed_paths: %d self-test case(s) failed\n' "$fails" >&2
        exit 1
    fi
    printf 'ci_changed_paths: self-test OK\n'
}

case "${1:-}" in
    --self-test) self_test ;;
    "")          classify ;;
    *)           printf 'ci_changed_paths: unknown argument %s\n' "$1" >&2; exit 2 ;;
esac
