#!/bin/sh
# Stage the agent harness for a release package.
#
# Usage: stage_harness.sh TREE DEST
#        stage_harness.sh --self-test
#
# Copies TREE, the repository's harness/ folder, to DEST, which must not exist
# yet, leaving out what serves development only: AGENTS.md and CLAUDE.md, the
# instruction files for coding agents, at any depth; and at the top of the
# tree tests/, .gitignore and .dockerignore. TREE must hold regular files and
# directories only: a symbolic link or any other entry is refused before
# anything is written. Group and other write are removed from every staged
# entry, because `cerulion` takes the harness beside its binary only when no
# user but its owner can write that folder. Files keep their executable bits.
#
# The release archive (release-artifacts.yml) and the Debian smoke (ci.yml)
# stage the harness with this script, and the install smoke compares an
# installed harness with what it stages, so the list of left out paths lives
# here alone.

set -eu

# Left out at the top of the tree.
TOP_EXCLUDED='tests .gitignore .dockerignore'
# Left out wherever they are.
ANY_DEPTH_EXCLUDED='AGENTS.md CLAUDE.md'

die() {
    printf 'error: %s\n' "$*" >&2
    exit 1
}

stage() {
    tree=$1
    dest=$2
    if [ ! -d "$tree" ] || [ -L "$tree" ]; then
        die "no harness directory at $tree"
    fi
    if [ -e "$dest" ] || [ -L "$dest" ]; then
        die "the staging destination already exists: $dest"
    fi
    odd=$(find "$tree" ! -type f ! -type d -print) ||
        die "could not read $tree"
    if [ -n "$odd" ]; then
        printf 'error: %s holds entries that are not regular files or directories:\n%s\n' \
            "$tree" "$odd" >&2
        exit 1
    fi
    cp -R "$tree" "$dest"
    for name in $TOP_EXCLUDED; do
        rm -rf "${dest:?}/$name"
    done
    for name in $ANY_DEPTH_EXCLUDED; do
        find "$dest" -name "$name" -type f -exec rm -f {} +
    done
    chmod -R go-w "$dest"
}

# Every left out path is absent from the stage, everything else arrives with
# its bytes and its executable bit, nothing is writable by group or others,
# and each refusal leaves no destination behind.
self_test() {
    work=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-stage-harness-test.XXXXXX")
    trap 'rm -rf "$work"' EXIT
    tree="$work/harness"
    mkdir -p "$tree/tests/fixtures" "$tree/measured/rcp" "$tree/src/tests"
    printf '%s\n' 'launcher' > "$tree/rcp.sh"
    printf '%s\n' 'install' > "$tree/install.sh"
    printf '%s\n' 'readme' > "$tree/README.md"
    printf '%s\n' 'measured' > "$tree/measured/rcp/cell.py"
    printf '%s\n' 'nested tests stay' > "$tree/src/tests/keep.py"
    printf '%s\n' 'agents' > "$tree/AGENTS.md"
    printf '%s\n' 'claude' > "$tree/CLAUDE.md"
    printf '%s\n' 'nested agents' > "$tree/measured/AGENTS.md"
    printf '%s\n' 'nested claude' > "$tree/measured/rcp/CLAUDE.md"
    printf '%s\n' 'test' > "$tree/tests/test_x.py"
    printf '%s\n' 'fixture' > "$tree/tests/fixtures/f.json"
    printf '%s\n' '*.pyc' > "$tree/.gitignore"
    printf '%s\n' 'tests' > "$tree/.dockerignore"
    chmod 0777 "$tree" "$tree/measured" "$tree/measured/rcp" "$tree/src" "$tree/src/tests"
    chmod 0777 "$tree/rcp.sh" "$tree/install.sh"
    chmod 0666 "$tree/README.md" "$tree/measured/rcp/cell.py" "$tree/src/tests/keep.py"

    dest="$work/stage/harness"
    mkdir -p "$work/stage"
    stage "$tree" "$dest"
    for gone in AGENTS.md CLAUDE.md measured/AGENTS.md measured/rcp/CLAUDE.md \
        tests .gitignore .dockerignore; do
        if [ -e "$dest/$gone" ] || [ -L "$dest/$gone" ]; then
            die "self-test: the stage kept $gone"
        fi
    done
    for kept in rcp.sh install.sh README.md measured/rcp/cell.py src/tests/keep.py; do
        cmp -s "$tree/$kept" "$dest/$kept" ||
            die "self-test: the stage lost or changed $kept"
    done
    for executable in rcp.sh install.sh; do
        [ -x "$dest/$executable" ] ||
            die "self-test: the stage dropped the executable bit of $executable"
    done
    if [ -x "$dest/README.md" ]; then
        die "self-test: the stage made README.md executable"
    fi
    writable=$(find "$dest" \( -perm -0020 -o -perm -0002 \) -print)
    [ -z "$writable" ] ||
        die "self-test: group or others can write the staged $writable"
    # The staged tree holds exactly the kept paths, so nothing new is shipped
    # by accident.
    listing=$(cd "$dest" && find . -type f | LC_ALL=C sort | tr '\n' ' ')
    [ "$listing" = "./README.md ./install.sh ./measured/rcp/cell.py ./rcp.sh ./src/tests/keep.py " ] ||
        die "self-test: the stage holds $listing"

    # Refusals: a symbolic link in the tree, a destination that exists, a
    # missing tree and a tree that is a link. None writes a destination.
    ln -s README.md "$tree/link"
    if (stage "$tree" "$work/stage/linked") 2>"$work/link-error"; then
        die "self-test: a tree holding a symbolic link was staged"
    fi
    grep -Fq 'not regular files or directories' "$work/link-error" ||
        die "self-test: the link refusal had the wrong diagnostic: $(cat "$work/link-error")"
    [ ! -e "$work/stage/linked" ] ||
        die "self-test: the link refusal wrote its destination"
    rm "$tree/link"
    if (stage "$tree" "$dest") 2>"$work/exists-error"; then
        die "self-test: an existing destination was staged over"
    fi
    grep -Fq 'already exists' "$work/exists-error" ||
        die "self-test: the existing destination refusal had the wrong diagnostic"
    if (stage "$work/absent" "$work/stage/absent") 2>"$work/absent-error"; then
        die "self-test: a missing tree was staged"
    fi
    ln -s "$tree" "$work/tree-link"
    if (stage "$work/tree-link" "$work/stage/tree-link") 2>"$work/tree-link-error"; then
        die "self-test: a tree that is a symbolic link was staged"
    fi
    for refused in absent tree-link; do
        grep -Fq 'no harness directory' "$work/$refused-error" ||
            die "self-test: the $refused refusal had the wrong diagnostic"
        [ ! -e "$work/stage/$refused" ] ||
            die "self-test: the $refused refusal wrote its destination"
    done
    printf '%s\n' 'stage_harness self-test passed'
}

if [ "$#" -eq 1 ] && [ "$1" = --self-test ]; then
    self_test
    exit 0
fi
if [ "$#" -ne 2 ]; then
    printf '%s\n' 'usage: stage_harness.sh TREE DEST | stage_harness.sh --self-test' >&2
    exit 2
fi
stage "$1" "$2"
