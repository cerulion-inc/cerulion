#!/usr/bin/env bash
# ci_changed_paths.sh: classify a pull request's changed paths for ci.yml.
#
# Reads one path per line on stdin and writes the four classes as
# `<class>=<value>` lines on stdout, one per line, alongside a `selection:` line
# that names the rule that fired. The caller reads the classes BY NAME, which is
# how ci.yml invokes it:
#
#   git diff --name-only --no-renames "$BASE...HEAD" > "$RUNNER_TEMP/changed.txt"
#   ./tools/scripts/ci_changed_paths.sh < "$RUNNER_TEMP/changed.txt" \
#     > "$RUNNER_TEMP/classes.txt"
#   code=$(sed -n 's/^code=//p' "$RUNNER_TEMP/classes.txt")
#
# and then writes each of `packaging`, `code`, `docs` and `pkgs` to
# `$GITHUB_OUTPUT` under its own literal name. Reading by name is what keeps the
# `selection:` lines OUT of `$GITHUB_OUTPUT`: they are for the reader of the job
# log, they carry no `<name>=<value>` shape, and piping this whole stream into
# the outputs file would declare an output nothing reads.
#
# `$BASE` above is the tip the pull request was merged onto whenever the event's
# `base.sha` is an ancestor of the checked-out merge commit's first parent, and
# ci.yml takes the base from that first parent. Otherwise `$BASE` is `base.sha`
# itself.
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

# A path that changes how EVERY package builds or is tested, and WHICH rule
# says so.
#
# The rule NAME is the point. Every one of these paths also selects everything
# through the unknown-path fallback at the bottom of `classify_touched`, so
# deleting a rule changes no answer and no self-test row could ever fail:
# mutant 20 was an EQUIVALENT mutant. The name reaches the caller, the caller
# prints it as a reason token, and each self-test row pins that token, so
# deleting a rule now changes the reason and the row fails.
workspace_input_rule() {
    local path=$1
    case "$path" in
        Cargo.lock) printf 'Cargo.lock\n'; return 0 ;;
        clippy.toml) printf 'clippy.toml\n'; return 0 ;;
        deny.toml) printf 'deny.toml\n'; return 0 ;;
        rust-toolchain|rust-toolchain.*) printf 'rust-toolchain\n'; return 0 ;;
        Cargo.toml|*/Cargo.toml) printf 'Cargo.toml\n'; return 0 ;;
        build.rs|*/build.rs) printf 'build.rs\n'; return 0 ;;
        .cargo/*) printf '.cargo\n'; return 0 ;;
        .config/*) printf '.config\n'; return 0 ;;
        .github/workflows/*) printf '.github/workflows\n'; return 0 ;;
        tools/*) printf 'tools\n'; return 0 ;;
        test_fixtures/*|*/test_fixtures/*) printf 'test_fixtures\n'; return 0 ;;
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
    local docs=false everything=false path owner name root entry rule
    local touched=" " reasons=""
    local switch=${CI_SELECTION:-on}

    # One token per RULE that fired, so the log line and every self-test row
    # name the rule rather than the answer. Two rules that agree on the answer
    # (and every workspace-level rule agrees with the unknown-path fallback)
    # are told apart by nothing else.
    # ONE REASON PER LINE, never space separated: a reason token carries a
    # changed path (`docs:My Notes.md`), and a space-separated accumulator
    # splits that into two tokens the moment the join expands it.
    note() {
        case "
$reasons" in
            *"
$1
"*) ;;
            *) reasons="$reasons$1
" ;;
        esac
    }
    touch_package() {
        case "$touched" in
            *" $1 "*) ;;
            *) touched="$touched$1 " ;;
        esac
    }

    if [ "${GITHUB_EVENT_NAME:-}" != "pull_request" ]; then
        everything=true
        note "event:${GITHUB_EVENT_NAME:-none}"
    elif [ "$switch" != "on" ]; then
        # `off` is the documented stop; any other value is a switch this script
        # cannot read, and an unreadable switch selects everything.
        everything=true
        note "switch:$switch"
    fi

    while IFS= read -r path; do
        [ -n "$path" ] || continue
        $everything && continue
        if rule=$(workspace_input_rule "$path"); then
            everything=true
            note "workspace-input:$rule"
            continue
        fi
        while IFS= read -r entry; do
            [ -n "$entry" ] || continue
            case "$path" in
                "${entry%%|*}"*)
                    docs=true
                    root=${entry##*|}
                    note "docs:$root"
                    while IFS= read -r name; do
                        [ -n "$name" ] || continue
                        touch_package "$name"
                    done < <(marker_packages_for_root "$root")
                    ;;
            esac
        done <<< "$SHARED_TREES"
        case "$path" in
            */*)
                case "$path" in
                    *.md) docs=true; note "docs:md" ;;
                esac
                ;;
            *.md)
                # A root markdown file: the markers name it by its own name.
                docs=true
                note "docs:$path"
                while IFS= read -r name; do
                    [ -n "$name" ] || continue
                    touch_package "$name"
                done < <(marker_packages_for_root "$path")
                ;;
        esac
        if owner=$(owning_manifest_dir "$path"); then
            case "$owner" in
                crates/*)
                    if name=$(manifest_package_name "$ROOT/$owner/Cargo.toml"); then
                        touch_package "$name"
                        note "crates:$name"
                    else
                        everything=true
                        note "unnamed-manifest"
                    fi
                    ;;
                *)
                    # A crate outside `crates/`: a benches workspace of its own,
                    # or the example node crate the viz library reaches through
                    # a path dev-dependency. A path cannot say which.
                    everything=true
                    note "foreign-workspace"
                    ;;
            esac
            continue
        fi
        case "$path" in
            crates/*) everything=true; note "unowned-crate"; continue ;;
            docs/*|benches/*|examples/*) continue ;;
            *.md) continue ;;
        esac
        everything=true
        note "unknown-path"
    done

    # QUOTED, so a reason naming a path with a space stays ONE token. The
    # accumulator is newline separated, so the sort and the join need no word
    # splitting and no pathname expansion to find the tokens.
    reasons=$(printf '%s' "$reasons" | sort -u | paste -sd, -)
    [ -n "$reasons" ] || reasons=none
    if $everything; then
        printf 'code=true\ndocs=true\ntouched=%s\nreason=%s\n' "$EVERY" "$reasons"
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
    printf 'docs=%s\ntouched=%s\nreason=%s\n' "$docs" "$touched" "$reasons"
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
    local packaging=false path lines code docs touched reason pkgs
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
    reason=$(printf '%s\n' "$lines" | sed -n 's/^reason=//p')
    pkgs=$(select_packages "$touched")

    # THE REASON, on its own line with the fixed `selection:` prefix, and NOT as
    # a `<name>=<value>` output: it is for the reader of the log, and the step
    # that consumes this writes only the four named classes to `$GITHUB_OUTPUT`.
    printf 'selection: reason=%s\n' "$reason"
    printf 'packaging=%s\n' "$packaging"
    printf 'code=%s\n' "$code"
    printf 'docs=%s\n' "$docs"
    printf 'pkgs=%s\n' "$pkgs"
}

self_test() {
    local fails=0
    # EVERY CASE THIS SUITE JUDGES, counted in one place: the packaging rows,
    # the class rows, the shared-tree loop, the live pipeline rows and the two
    # emergency-stop rows. The count is the witness the last line carries, and
    # it is what the nested run below is judged on, so a suite that stops
    # judging rows reports a smaller number rather than the same `OK`.
    local cases=0

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
        cases=$((cases + 1))
        paths=${line%|*}
        want=${line##*|}
        got=$(printf '%s\n' "${paths//;/$'\n'}" \
              | GITHUB_EVENT_NAME=push CI_SELECTION=on classify \
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
    # `<event>;<switch>;<paths>|<code>|<docs>|<touched>|<reason>`, `,` separating
    # paths. Every row is written out by hand: the expected answer says what the
    # rule IS, so a rule that changes its mind fails here rather than agreeing
    # with itself.
    #
    # THE REASON IS THE FOURTH FIELD and it is what makes the workspace-level
    # rows able to fail. Every one of those paths also selects everything
    # through the unknown-path fallback, so `all` alone is satisfied by deleting
    # the rule; the reason names the rule that fired, so deleting it changes the
    # reason and the row reds.
    #
    # A ROOT MARKDOWN PATH WITH A SPACE has a row of its own, because the reason
    # is the one field a changed path reaches verbatim: the join has to keep
    # `docs:My Notes.md` as ONE token, and an unquoted expansion splits it into
    # two and then globs whichever half looks like a pattern.
    #
    # THE SWITCH IS PINNED ON EVERY ROW, empty for "unset". Inheriting it from
    # the environment made the whole table answer `all` under the repository
    # variable `CI_SELECTION=off`, which is the emergency stop reddening the
    # required Lint context on every event.
    #
    # The event rows name a path a manifest OWNS. They used to name
    # `crates/cerulion_vizd/src/lib.rs`, which no manifest owns, so they
    # answered `all` through the unknown-path fallback whatever the event rule
    # did and the merge_group mutant survived them.
    local selection_cases='
push;;crates/cerulion_viz/bin/cerulion_vizd/src/main.rs|true|true|all|event:push
merge_group;;crates/cerulion_viz/bin/cerulion_vizd/src/main.rs|true|true|all|event:merge_group
workflow_dispatch;;crates/cerulion_viz/bin/cerulion_vizd/src/main.rs|true|true|all|event:workflow_dispatch
schedule;;crates/cerulion_viz/bin/cerulion_vizd/src/main.rs|true|true|all|event:schedule
pull_request;off;crates/cerulion_viz/bin/cerulion_vizd/src/main.rs|true|true|all|switch:off
pull_request;maybe;crates/cerulion_viz/bin/cerulion_vizd/src/main.rs|true|true|all|switch:maybe
pull_request;on;crates/cerulion_viz/bin/cerulion_vizd/src/main.rs|true|false|cerulion_vizd|crates:cerulion_vizd
pull_request;;crates/cerulion_viz/bin/cerulion_vizd/src/main.rs|true|false|cerulion_vizd|crates:cerulion_vizd
pull_request;;Cargo.lock|true|true|all|workspace-input:Cargo.lock
pull_request;;Cargo.toml|true|true|all|workspace-input:Cargo.toml
pull_request;;crates/cerulion_core/Cargo.toml|true|true|all|workspace-input:Cargo.toml
pull_request;;.cargo/config.toml|true|true|all|workspace-input:.cargo
pull_request;;.config/nextest.toml|true|true|all|workspace-input:.config
pull_request;;rust-toolchain.toml|true|true|all|workspace-input:rust-toolchain
pull_request;;crates/cerulion_core/build.rs|true|true|all|workspace-input:build.rs
pull_request;;clippy.toml|true|true|all|workspace-input:clippy.toml
pull_request;;deny.toml|true|true|all|workspace-input:deny.toml
pull_request;;.github/workflows/ci.yml|true|true|all|workspace-input:.github/workflows
pull_request;;tools/scripts/ci_selected_packages.py|true|true|all|workspace-input:tools
pull_request;;tools/ci/observation_edges.tsv|true|true|all|workspace-input:tools
pull_request;;crates/test_fixtures/test_node_cdylib/src/lib.rs|true|true|all|workspace-input:test_fixtures
pull_request;;crates/cerulion_core/src/wire.rs|true|false|cerulion_core|crates:cerulion_core
pull_request;;crates/cerulion_viz/lib/go2_tf/src/lib.rs|true|false|go2_tf|crates:go2_tf
pull_request;;crates/cerulion_core/src/wire.rs,crates/cerulion_bag/src/lib.rs|true|false|cerulion_bag cerulion_core|crates:cerulion_bag,crates:cerulion_core
pull_request;;crates/cerulion_core/AGENTS.md|true|true|cerulion_core|crates:cerulion_core,docs:md
pull_request;;crates/cerulion_core/notes.txt|true|false|cerulion_core|crates:cerulion_core
pull_request;;crates/nothing_owns_this.txt|true|true|all|unowned-crate
pull_request;;examples/go2/nodes/go2_tf_source/src/lib.rs|true|true|all|docs:examples,foreign-workspace
pull_request;;benches/latency/workspace/nodes/ping_node/src/lib.rs|true|true|all|docs:benches,foreign-workspace
pull_request;;LICENSE|true|true|all|unknown-path
pull_request;;.github/CODEOWNERS|true|true|all|unknown-path
pull_request;;CITATION.cff|true|true|all|unknown-path
pull_request;;README.md|true|true|cerulion_cli_engine|docs:README.md
pull_request;;My Notes.md|false|true||docs:My Notes.md
pull_request;;|false|false||none
'
    local event switch head
    while IFS= read -r line; do
        [ -n "$line" ] || continue
        cases=$((cases + 1))
        head=${line%%|*}
        event=${head%%;*}
        head=${head#*;}
        switch=${head%%;*}
        paths=${head#*;}
        want=${line#*|}
        got=$(printf '%s\n' "${paths//,/$'\n'}" \
              | GITHUB_EVENT_NAME=$event CI_SELECTION=$switch classify_touched \
              | sed -e 's/^code=//' -e 's/^docs=//' -e 's/^touched=//' -e 's/^reason=//' \
              | paste -sd'|' -)
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
    local tree root markers
    for tree in docs benches examples; do
        cases=$((cases + 1))
        got=$(printf '%s/guide.md\n' "$tree" \
              | GITHUB_EVENT_NAME=pull_request CI_SELECTION=on classify_touched \
              | sed -n 's/^touched=//p')
        if [ "$got" = "$EVERY" ] || [ -z "$got" ]; then
            printf 'ci_changed_paths self-test: %s/ -> touched=%s, wanted the packages whose doc-pin markers read %s\n' \
                "$tree" "${got:-<empty>}" "$tree" >&2
            fails=$((fails + 1))
        fi
        # Read the markers ONCE and match without a pipe: `grep -q` closes the
        # pipe on its first hit, `pipefail` then reports the producer's SIGPIPE,
        # and every package that DID match was reported as a miss.
        markers=$(marker_packages_for_root "$tree")
        for name in $got; do
            cases=$((cases + 1))
            case "
$markers
" in
                *"
$name
"*) ;;
                *)
                    printf 'ci_changed_paths self-test: %s/ touched %s, which no doc-pin marker names as reading %s\n' \
                        "$tree" "$name" "$tree" >&2
                    fails=$((fails + 1))
                    ;;
            esac
        done
    done
    # The other side: a tree with no marker at all touches nothing, so the rule
    # cannot be satisfied by a reader that returns every package for any input.
    cases=$((cases + 1))
    got=$(marker_packages_for_root "no_such_tree" | tr '\n' ' ')
    if [ -n "${got// /}" ]; then
        printf 'ci_changed_paths self-test: a tree no marker names read back %s\n' "$got" >&2
        fails=$((fails + 1))
    fi
    # A root markdown file reaches the markers by its own name, and the four the
    # walk knows are the four `git ls-files` reports at the root.
    cases=$((cases + 1))
    got=$(printf 'AGENTS.md\n' \
          | GITHUB_EVENT_NAME=pull_request CI_SELECTION=on classify_touched \
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
    cases=$((cases + 1))
    got=$(printf 'crates/cerulion_viz/bin/cerulion_vizd/src/main.rs\n' \
          | GITHUB_EVENT_NAME=pull_request CI_SELECTION=on classify \
          | sed -n 's/^pkgs=//p')
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
    cases=$((cases + 1))
    case "$got" in
        *'"cerulion_bagd"'*)
            printf 'ci_changed_paths self-test: a vizd-only change selected cerulion_bagd: %s\n' \
                "$got" >&2
            fails=$((fails + 1))
            ;;
    esac
    # A workspace-level input selects every member, spelled as the array, never
    # as the word `all`: `fromJSON` on that word is an expression error.
    cases=$((cases + 1))
    got=$(printf '.github/workflows/ci.yml\n' \
          | GITHUB_EVENT_NAME=pull_request CI_SELECTION=on classify \
          | sed -n 's/^pkgs=//p')
    case "$got" in
        *'"cerulion_vizd"'*'"cerulion_core"'*|*'"cerulion_core"'*'"cerulion_vizd"'*) ;;
        *)
            printf 'ci_changed_paths self-test: a workflow edit -> pkgs=%s, wanted the array of every member\n' \
                "$got" >&2
            fails=$((fails + 1))
            ;;
    esac

    # ---- THE EMERGENCY STOP DOES NOT BREAK THIS GATE ---------------------
    # `lint` runs this self-test with the workflow-level environment, so the
    # repository variable `CI_SELECTION=off` reaches it. Every arm above pins
    # the switch, and this arm proves it by running the WHOLE suite again with
    # the variable set: turning the selection off must red nothing.
    #
    # Once, guarded by its own variable, because the nested run reaches this
    # line too.
    #
    # BOTH ARMS READ THE NESTED RUN'S CASE COUNT, never its exit status. A run
    # that returns early on the guard variable exits 0 as well, so an arm that
    # discards the output and keeps the status passes a process that judged no
    # row. The nested run skips this block and nothing else, so it judges
    # exactly the cases this run has judged on reaching here: that number is
    # the witness, and a run that prints no count line, a different count, or
    # `OK` with no count at all reds the arm.
    if [ -z "${CI_CHANGED_PATHS_NESTED_SELF_TEST:-}" ]; then
        local nested_point=$cases nested_out
        # `$1` names the run for the message, `$2` is its captured stdout.
        judge_nested() {
            local count
            count=$(printf '%s\n' "$2" \
                    | sed -n 's/^ci_changed_paths: self-test OK (\([0-9][0-9]*\) cases)$/\1/p')
            if [ -z "$count" ]; then
                printf 'ci_changed_paths self-test: the nested run %s printed no self-test OK (<n> cases) line, so nothing says it judged a row\n' \
                    "$1" >&2
                fails=$((fails + 1))
                return 0
            fi
            if [ "$count" != "$nested_point" ]; then
                printf 'ci_changed_paths self-test: the nested run %s judged %s case(s), and this run had judged %s on reaching it\n' \
                    "$1" "$count" "$nested_point" >&2
                fails=$((fails + 1))
                return 0
            fi
            # The floor stands on its own, so a suite that loses most of its
            # table in both processes at once still reds here.
            if [ "$count" -lt 60 ]; then
                printf 'ci_changed_paths self-test: the nested run %s judged %s case(s), under the floor of 60\n' \
                    "$1" "$count" >&2
                fails=$((fails + 1))
            fi
        }
        if nested_out=$(CI_SELECTION=off CI_CHANGED_PATHS_NESTED_SELF_TEST=1 \
                "$ROOT/tools/scripts/ci_changed_paths.sh" --self-test 2>/dev/null); then
            judge_nested 'under CI_SELECTION=off' "$nested_out"
        else
            printf 'ci_changed_paths self-test: the suite fails under CI_SELECTION=off, so the emergency stop reds the Lint job\n' >&2
            fails=$((fails + 1))
        fi
        # The other side, with the switch REALLY unset: `env -u` removes it, so
        # this arm differs from the one above on every event. Inheriting it made
        # the two runs the same command under the repository variable
        # `CI_SELECTION=off`, which is the one state the block exists for.
        if nested_out=$(env -u CI_SELECTION CI_CHANGED_PATHS_NESTED_SELF_TEST=1 \
                "$ROOT/tools/scripts/ci_changed_paths.sh" --self-test 2>/dev/null); then
            judge_nested 'with the switch unset' "$nested_out"
        else
            printf 'ci_changed_paths self-test: the suite fails with the switch unset, so the arm above tells nothing apart\n' >&2
            fails=$((fails + 1))
        fi
        cases=$((cases + 2))
    fi

    if [ "$fails" -ne 0 ]; then
        printf 'ci_changed_paths: %d self-test case(s) failed\n' "$fails" >&2
        exit 1
    fi
    printf 'ci_changed_paths: self-test OK (%d cases)\n' "$cases"
}

case "${1:-}" in
    --self-test) self_test ;;
    "")          classify ;;
    *)           printf 'ci_changed_paths: unknown argument %s\n' "$1" >&2; exit 2 ;;
esac
