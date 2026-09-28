#!/usr/bin/env bash
# ci_changed_paths.sh: classify a pull request's changed paths for ci.yml.
#
# Reads one path per line on stdin and writes `<class>=<value>` lines on stdout,
# in the shape `$GITHUB_OUTPUT` wants:
#
#   git diff --name-only "$BASE" HEAD | tools/scripts/ci_changed_paths.sh >> "$GITHUB_OUTPUT"
#
# FOUR CLASSES, and two directions. `packaging` only ever ADDS work: it runs a
# push-only job on a pull request that touches its inputs. `code`, `docs` and
# `pkgs` do the opposite: ci.yml gates test steps on them, so a rule that is too
# NARROW skips a test the change could have broken, and skips it silently. Every
# rule below is therefore fail-closed: an unclassifiable path, an event that is
# not a pull request, a switch this script cannot read, or a selector that
# refuses, all select EVERYTHING.
#
# `packaging` is the inputs of the `deb-smoke` job (Debian and APT package
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
# `code`, `docs`, `pkgs`: the test-impact selection. THE RULES, IN THIS ORDER.
#
#   1. Not a `pull_request` event. Selection gates pull-request runs ONLY.
#      `merge_group` and `push` runs on `main` stay full: the queue run is the
#      last gate before `main` and the one place a miss has no later catch.
#   2. `CI_SELECTION` is `off`, or carries any value this script does not know.
#      One repository variable stops an outward misfire with no pull request.
#   3. A WORKSPACE-LEVEL INPUT. Any `Cargo.toml` (root or member: feature
#      unification and `default-members` change another package's build without
#      touching it), `Cargo.lock`, `.cargo/**`, `.config/**`, `rust-toolchain*`,
#      any `build.rs`, `clippy.toml`, `deny.toml`, `.github/workflows/**`,
#      `tools/**` (which holds this script, the selector, the shard runner and
#      the committed observation-edge table), and any `test_fixtures/**` path
#      (the dlopen fixtures: their artifact names are built at run time, so no
#      derived edge can attribute one).
#   4. A path under a workspace member's directory: that member is TOUCHED. The
#      owning member is the longest directory prefix carrying a `Cargo.toml`,
#      read off the filesystem, so a nested member (`crates/cerulion_viz/lib/
#      go2_tf`) wins over the directory above it. A `crates/` path no manifest
#      owns selects everything.
#   5. A SHARED TREE: `docs/**`, `benches/**`, `examples/**`, or any `*.md`.
#      `docs=true`, and every package the doc-pin markers in ci.yml name as
#      READING that tree is touched. Those markers are the derived, two-sidedly
#      pinned table `cerulion_cli_engine::ci_doc_pin_walk_test` holds to the test
#      sources, so this rule cannot go stale on its own. A `benches/` or
#      `examples/` path that a `Cargo.toml` owns selects everything: those trees
#      carry crates (one of them a workspace member reached through a path
#      dev-dependency), and a path alone cannot say which workspace they belong
#      to.
#   6. Anything else selects everything.
#
# `pkgs` is the selector's answer over the touched set: the reverse cargo
# dependency closure UNIONED with the observation edges
# (`tools/ci/observation_edges.tsv`), ALWAYS as a JSON array. The whole-workspace
# answer is the array of every member rather than the word `all`, because the
# only reader is `contains(fromJSON(needs.changes.outputs.pkgs), '<pkg>')` and
# `fromJSON` on that word is an expression error: every gated step would fail
# instead of running. A selector that refuses falls back to that array.
#
# `code` follows the selection being non-empty, which is the reading
# `cerulion_cli_engine::ci_test_coverage_test` holds every gated step to.
#
# Renames reach here as a DELETE plus an ADD, never as a destination path alone:
# the workflow diffs with `--no-renames` so that renaming a listed input away
# still matches on the name it had.
#
# Unreadable or empty input yields `packaging=false`; the selection classes then
# read the empty change as what it is, `code=false` and `pkgs=[]`, on a pull
# request and every member on everything else.
#
# `--self-test` runs the tables below and exits nonzero on the first miss.
set -euo pipefail

ROOT=$(cd "$(dirname "$0")/../.." && pwd)

# The workflow whose doc-pin markers rule 5 reads.
CI_WORKFLOW_REL=".github/workflows/ci.yml"
# The derived observation-edge table the selector unions into the closure.
OBSERVATION_EDGES_REL="tools/ci/observation_edges.tsv"
# The selector.
SELECTOR_REL="tools/scripts/ci_selected_packages.py"

# The sentinel that means "every package", in the touched set and in `pkgs`.
EVERY=all

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

# The shared trees rule 5 names, one per line, as `<prefix>|<doc-pin root>`.
SHARED_TREES='
docs/|docs
benches/|benches
examples/|examples
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

# A path that changes how EVERY package builds or is tested.
matches_workspace_input() {
    local path=$1
    case "$path" in
        Cargo.lock|clippy.toml|deny.toml) return 0 ;;
        rust-toolchain|rust-toolchain.*) return 0 ;;
        Cargo.toml|*/Cargo.toml) return 0 ;;
        build.rs|*/build.rs) return 0 ;;
        .cargo/*|.config/*|.github/workflows/*|tools/*) return 0 ;;
        test_fixtures/*|*/test_fixtures/*) return 0 ;;
    esac
    return 1
}

# The workspace member directory that owns `$1`, or the empty string.
#
# The longest directory prefix of the path that carries a `Cargo.toml`. Read off
# the filesystem rather than from a list, so a member added, moved or nested
# under another one is owned the moment it exists.
owning_manifest_dir() {
    local dir=${1%/*}
    while [ -n "$dir" ] && [ "$dir" != "$1" ]; do
        if [ -f "$ROOT/$dir/Cargo.toml" ]; then
            printf '%s\n' "$dir"
            return 0
        fi
        case "$dir" in
            */*) dir=${dir%/*} ;;
            *)   dir="" ;;
        esac
    done
    return 1
}

# The `name = "..."` a crate manifest declares.
#
# Stops at the table after `[package]`, so a `name` under `[[bin]]` is not the
# package's.
manifest_package_name() {
    local manifest=$1 line rest
    while IFS= read -r line; do
        line=${line#"${line%%[![:space:]]*}"}
        case "$line" in
            name*=*)
                rest=${line#*=}
                rest=${rest#"${rest%%[![:space:]]*}"}
                case "$rest" in
                    \"*) rest=${rest#\"}; printf '%s\n' "${rest%%\"*}"; return 0 ;;
                esac
                ;;
            '[package]') ;;
            \[*) return 1 ;;
        esac
    done < "$manifest"
    return 1
}

# Every package a doc-pin marker in ci.yml names as READING the tree `$1`.
marker_packages_for_root() {
    local root=$1 line pair roots
    [ -f "$ROOT/$CI_WORKFLOW_REL" ] || return 0
    while IFS= read -r line; do
        line=${line#*"# doc-pin: "}
        case "$line" in
            *" reads "*) ;;
            *) continue ;;
        esac
        pair=${line%% reads *}
        roots=${line#* reads }
        case ",${roots// /}," in
            *",$root,"*) printf '%s\n' "${pair%%::*}" ;;
        esac
    done < <(grep -F '# doc-pin: ' "$ROOT/$CI_WORKFLOW_REL" || true)
}

# Read one path per line; print `code=`, `docs=` and `touched=` lines.
#
# `touched=` is this script's own intermediate: either `all` or a
# space-separated list of package names, which `classify` turns into `pkgs`.
# Kept separate so the self-test can drive the RULES without a cargo resolve.
classify_touched() {
    local docs=false everything=false path owner name root entry
    local touched=" "
    local switch=${CI_SELECTION:-on}

    if [ "${GITHUB_EVENT_NAME:-}" != "pull_request" ]; then
        everything=true
    elif [ "$switch" != "on" ]; then
        # `off` is the documented stop; any other value is a switch this script
        # cannot read, and an unreadable switch selects everything.
        everything=true
    fi

    while IFS= read -r path; do
        [ -n "$path" ] || continue
        $everything && continue
        if matches_workspace_input "$path"; then
            everything=true
            continue
        fi
        case "$path" in
            *.md) docs=true ;;
        esac
        while IFS= read -r entry; do
            [ -n "$entry" ] || continue
            case "$path" in
                "${entry%%|*}"*)
                    docs=true
                    root=${entry##*|}
                    while IFS= read -r name; do
                        [ -n "$name" ] || continue
                        case "$touched" in
                            *" $name "*) ;;
                            *) touched="$touched$name " ;;
                        esac
                    done < <(marker_packages_for_root "$root")
                    ;;
            esac
        done <<< "$SHARED_TREES"
        case "$path" in
            */*) ;;
            *.md)
                # A root markdown file: the markers name it by its own name.
                while IFS= read -r name; do
                    [ -n "$name" ] || continue
                    case "$touched" in
                        *" $name "*) ;;
                        *) touched="$touched$name " ;;
                    esac
                done < <(marker_packages_for_root "$path")
                ;;
        esac
        if owner=$(owning_manifest_dir "$path"); then
            case "$owner" in
                crates/*)
                    if name=$(manifest_package_name "$ROOT/$owner/Cargo.toml"); then
                        case "$touched" in
                            *" $name "*) ;;
                            *) touched="$touched$name " ;;
                        esac
                    else
                        everything=true
                    fi
                    ;;
                *)
                    # A crate outside `crates/`: a benches workspace of its own,
                    # or the example node crate the viz library reaches through
                    # a path dev-dependency. A path cannot say which.
                    everything=true
                    ;;
            esac
            continue
        fi
        case "$path" in
            docs/*|benches/*|examples/*) continue ;;
            *.md) continue ;;
        esac
        everything=true
    done

    if $everything; then
        printf 'code=true\ndocs=true\ntouched=%s\n' "$EVERY"
        return 0
    fi
    # Sorted and deduplicated, so the touched set is a function of the changed
    # paths and not of the order they arrived in: a log line and a `pkgs` value
    # a reader compares across two runs have to be the same string.
    # shellcheck disable=SC2086
    touched=$(printf '%s\n' $touched | sort -u | tr '\n' ' ')
    touched=${touched# }
    touched=${touched% }
    if [ -n "$touched" ]; then
        printf 'code=true\n'
    else
        printf 'code=false\n'
    fi
    printf 'docs=%s\ntouched=%s\n' "$docs" "$touched"
}

# The selector's answer for a touched set, ALWAYS as a JSON array.
#
# `pkgs` is read by `contains(fromJSON(needs.changes.outputs.pkgs), '<pkg>')`,
# and `fromJSON` on the word `all` is an EXPRESSION ERROR, not a false
# condition: every gated step would fail rather than run. So the whole-workspace
# answer is spelled as the array of every member, which makes every per-package
# condition true, and the word `all` stays where it belongs, in the touched set
# and in the log line a reader sees.
#
# A selector that refuses for any reason falls back to that same whole-workspace
# array. A workspace this script cannot even list is a REFUSAL: the classifier
# exits nonzero, the `changes` job goes red, and every job that needs it stops.
# An empty `pkgs` would read as "run nothing" on every gated step.
select_packages() {
    local touched=$1 out
    if [ -z "$touched" ]; then
        printf '[]\n'
        return 0
    fi
    if [ "$touched" != "$EVERY" ]; then
        # shellcheck disable=SC2086
        if out=$(cd "$ROOT" && cargo metadata --format-version 1 --no-deps 2>/dev/null \
                | python3 -B "$ROOT/$SELECTOR_REL" \
                    --observation-edges "$ROOT/$OBSERVATION_EDGES_REL" $touched 2>&1); then
            printf '%s\n' "$out"
            return 0
        fi
        printf 'ci_changed_paths: the selector refused, selecting everything: %s\n' "$out" >&2
    fi
    if out=$(cd "$ROOT" && cargo metadata --format-version 1 --no-deps 2>/dev/null \
            | python3 -B "$ROOT/$SELECTOR_REL" --all 2>&1); then
        printf '%s\n' "$out"
        return 0
    fi
    printf 'ci_changed_paths: cannot list the workspace members: %s\n' "$out" >&2
    return 1
}

classify() {
    local packaging=false path lines code docs touched pkgs
    local input
    input=$(cat)
    while IFS= read -r path; do
        [ -n "$path" ] || continue
        if matches_packaging "$path"; then
            packaging=true
        fi
    done <<< "$input"

    lines=$(printf '%s\n' "$input" | classify_touched)
    code=$(printf '%s\n' "$lines" | sed -n 's/^code=//p')
    docs=$(printf '%s\n' "$lines" | sed -n 's/^docs=//p')
    touched=$(printf '%s\n' "$lines" | sed -n 's/^touched=//p')
    pkgs=$(select_packages "$touched")

    printf 'packaging=%s\n' "$packaging"
    printf 'code=%s\n' "$code"
    printf 'docs=%s\n' "$docs"
    printf 'pkgs=%s\n' "$pkgs"
}

self_test() {
    local fails=0

    # ---- the packaging table, unchanged -----------------------------------
    # `<input paths>|<expected packaging>`; `;` separates paths.
    local packaging_cases='
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
        got=$(printf '%s\n' "${paths//;/$'\n'}" | GITHUB_EVENT_NAME=push classify \
              | sed -n 's/^packaging=//p')
        if [ "$got" != "$want" ]; then
            printf 'ci_changed_paths self-test: %s -> packaging=%s, wanted packaging=%s\n' \
                "${paths:-<empty>}" "$got" "$want" >&2
            fails=$((fails + 1))
        fi
    done <<< "$packaging_cases"

    # ---- the selection rules ----------------------------------------------
    # `<event>;<switch>;<paths>|<code>|<docs>|<touched>`, `,` separating paths.
    # Every row is written out by hand: the expected answer says what the rule
    # IS, so a rule that changes its mind fails here rather than agreeing with
    # itself. Each row names the mutant it kills in the comment above it.
    local selection_cases='
push;;crates/cerulion_vizd/src/lib.rs|true|true|all
merge_group;;crates/cerulion_vizd/src/lib.rs|true|true|all
workflow_dispatch;;crates/cerulion_vizd/src/lib.rs|true|true|all
schedule;;crates/cerulion_vizd/src/lib.rs|true|true|all
pull_request;off;crates/cerulion_viz/bin/cerulion_vizd/src/main.rs|true|true|all
pull_request;maybe;crates/cerulion_viz/bin/cerulion_vizd/src/main.rs|true|true|all
pull_request;on;crates/cerulion_viz/bin/cerulion_vizd/src/main.rs|true|false|cerulion_vizd
pull_request;;crates/cerulion_viz/bin/cerulion_vizd/src/main.rs|true|false|cerulion_vizd
pull_request;;Cargo.lock|true|true|all
pull_request;;Cargo.toml|true|true|all
pull_request;;crates/cerulion_core/Cargo.toml|true|true|all
pull_request;;.cargo/config.toml|true|true|all
pull_request;;.config/nextest.toml|true|true|all
pull_request;;rust-toolchain.toml|true|true|all
pull_request;;crates/cerulion_core/build.rs|true|true|all
pull_request;;clippy.toml|true|true|all
pull_request;;deny.toml|true|true|all
pull_request;;.github/workflows/ci.yml|true|true|all
pull_request;;tools/scripts/ci_selected_packages.py|true|true|all
pull_request;;tools/ci/observation_edges.tsv|true|true|all
pull_request;;crates/test_fixtures/test_node_cdylib/src/lib.rs|true|true|all
pull_request;;crates/cerulion_core/src/wire.rs|true|false|cerulion_core
pull_request;;crates/cerulion_viz/lib/go2_tf/src/lib.rs|true|false|go2_tf
pull_request;;crates/cerulion_core/src/wire.rs,crates/cerulion_bag/src/lib.rs|true|false|cerulion_bag cerulion_core
pull_request;;crates/cerulion_core/AGENTS.md|true|true|cerulion_core
pull_request;;crates/cerulion_core/notes.txt|true|false|cerulion_core
pull_request;;crates/nothing_owns_this.txt|true|true|all
pull_request;;examples/go2/nodes/go2_tf_source/src/lib.rs|true|true|all
pull_request;;benches/latency/workspace/nodes/ping_node/src/lib.rs|true|true|all
pull_request;;LICENSE|true|true|all
pull_request;;.github/CODEOWNERS|true|true|all
pull_request;;CITATION.cff|true|true|all
pull_request;;|false|false|
'
    local event switch head
    while IFS= read -r line; do
        [ -n "$line" ] || continue
        head=${line%%|*}
        event=${head%%;*}
        head=${head#*;}
        switch=${head%%;*}
        paths=${head#*;}
        want=${line#*|}
        got=$(printf '%s\n' "${paths//,/$'\n'}" \
              | GITHUB_EVENT_NAME=$event CI_SELECTION=$switch classify_touched \
              | sed -e 's/^code=//' -e 's/^docs=//' -e 's/^touched=//' | paste -sd'|' -)
        if [ "$got" != "$want" ]; then
            printf 'ci_changed_paths self-test: [%s %s] %s -> %s, wanted %s\n' \
                "$event" "${switch:-<unset>}" "${paths:-<empty>}" "$got" "$want" >&2
            fails=$((fails + 1))
        fi
    done <<< "$selection_cases"

    # ---- the shared trees, against the markers in the workflow -------------
    # The expected package sets are NOT typed out: they are what the derived,
    # two-sidedly pinned doc-pin markers say, and typing them here would be a
    # second hand list going stale beside the first. What IS asserted by hand is
    # the SHAPE: a documentation tree touches at least one package and never
    # every package, and the package it touches is one whose marker names that
    # tree.
    local tree root
    for tree in docs benches examples; do
        got=$(printf '%s/guide.md\n' "$tree" \
              | GITHUB_EVENT_NAME=pull_request classify_touched | sed -n 's/^touched=//p')
        if [ "$got" = "$EVERY" ] || [ -z "$got" ]; then
            printf 'ci_changed_paths self-test: %s/ -> touched=%s, wanted the packages whose doc-pin markers read %s\n' \
                "$tree" "${got:-<empty>}" "$tree" >&2
            fails=$((fails + 1))
        fi
        for name in $got; do
            if ! marker_packages_for_root "$tree" | grep -qx "$name"; then
                printf 'ci_changed_paths self-test: %s/ touched %s, which no doc-pin marker names as reading %s\n' \
                    "$tree" "$name" "$tree" >&2
                fails=$((fails + 1))
            fi
        done
    done
    # The other side: a tree with no marker at all touches nothing, so the rule
    # cannot be satisfied by a reader that returns every package for any input.
    got=$(marker_packages_for_root "no_such_tree" | tr '\n' ' ')
    if [ -n "${got// /}" ]; then
        printf 'ci_changed_paths self-test: a tree no marker names read back %s\n' "$got" >&2
        fails=$((fails + 1))
    fi
    # A root markdown file reaches the markers by its own name, and the four the
    # walk knows are the four `git ls-files` reports at the root.
    got=$(printf 'AGENTS.md\n' | GITHUB_EVENT_NAME=pull_request classify_touched \
          | sed -n 's/^docs=//p')
    if [ "$got" != "true" ]; then
        printf 'ci_changed_paths self-test: a root markdown file -> docs=%s, wanted true\n' \
            "$got" >&2
        fails=$((fails + 1))
    fi

    # ---- the whole pipeline, once, over the real workspace -----------------
    # Everything above stops at `touched`. This arm proves the wiring: the
    # selector is invoked, the observation-edge table is read, and `pkgs` comes
    # back as a JSON array that holds the touched package itself.
    got=$(printf 'crates/cerulion_viz/bin/cerulion_vizd/src/main.rs\n' \
          | GITHUB_EVENT_NAME=pull_request classify | sed -n 's/^pkgs=//p')
    case "$got" in
        *'"cerulion_vizd"'*) ;;
        *)
            printf 'ci_changed_paths self-test: the live pipeline -> pkgs=%s, wanted a JSON array holding cerulion_vizd\n' \
                "$got" >&2
            fails=$((fails + 1))
            ;;
    esac
    # The other side of the same answer: a vizd-only change does NOT select a
    # package nothing connects it to, so the narrow answer is narrow.
    case "$got" in
        *'"cerulion_bagd"'*)
            printf 'ci_changed_paths self-test: a vizd-only change selected cerulion_bagd: %s\n' \
                "$got" >&2
            fails=$((fails + 1))
            ;;
    esac
    # A workspace-level input selects every member, spelled as the array, never
    # as the word `all`: `fromJSON` on that word is an expression error.
    got=$(printf '.github/workflows/ci.yml\n' \
          | GITHUB_EVENT_NAME=pull_request classify | sed -n 's/^pkgs=//p')
    case "$got" in
        *'"cerulion_vizd"'*'"cerulion_core"'*|*'"cerulion_core"'*'"cerulion_vizd"'*) ;;
        *)
            printf 'ci_changed_paths self-test: a workflow edit -> pkgs=%s, wanted the array of every member\n' \
                "$got" >&2
            fails=$((fails + 1))
            ;;
    esac

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
