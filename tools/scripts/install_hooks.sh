#!/usr/bin/env bash
# install_hooks.sh: install the leak guard git hooks for this repository.
#
#   install_hooks.sh              copy tools/hooks and the scanner to an ABSOLUTE
#                                 directory under the shared git directory and
#                                 point the local core.hooksPath at it, so every
#                                 linked worktree on every branch is covered
#   install_hooks.sh --in-tree    point core.hooksPath at the relative tools/hooks
#                                 instead (one checkout; a worktree whose branch
#                                 lacks tools/hooks silently runs nothing)
#   install_hooks.sh --force      replace a core.hooksPath this script did not set
#   install_hooks.sh --status     show what is active
#   install_hooks.sh --uninstall  restore the value core.hooksPath had before
#   install_hooks.sh --self-test  prove the install works in a scratch repository
#
# Why a copy and not the relative path by default: a relative core.hooksPath is
# resolved against each worktree, and a worktree checked out at a branch that
# predates tools/hooks runs NO hook and reports nothing. Measured. The copy goes
# stale when the tree's hooks change; each installed hook says so and names
# this script.
#
# The hooks CHAIN to whatever core.hooksPath pointed at before (a global
# identity hook, the repository's own hooks directory), so installing the guard
# never silences another hook.
#
# Exit 0 = done. Exit 1 = a check failed (self-test, or a refused install).
# Exit 2 = malformed invocation. Portable: bash 3.2+ (macOS), BSD and GNU userland.

set -u

cd "$(dirname "$0")/../.." || exit 2

MODE=install
FORCE=no
while [ "$#" -gt 0 ]; do
    case "$1" in
        --in-tree) MODE=in-tree; shift ;;
        --force) FORCE=yes; shift ;;
        --status) MODE=status; shift ;;
        --uninstall) MODE=uninstall; shift ;;
        --self-test) MODE=self-test; shift ;;
        -h|--help)
            awk 'NR == 1 { next } /^#/ { print; next } { exit }' "$0"
            exit 0
            ;;
        *)
            printf 'install_hooks: unknown argument %s\n' "$1" >&2
            exit 2
            ;;
    esac
done
if [ "$MODE" = self-test ] && [ "$FORCE" = yes ]; then
    printf 'install_hooks: --self-test takes no other argument\n' >&2
    exit 2
fi

HOOK_FILES="commit-msg pre-commit leak-guard-lib.sh"

common_dir() {
    # Absolute path of the shared git directory (the one every linked worktree uses).
    _cd=$(git rev-parse --git-common-dir 2>/dev/null) || return 1
    (cd "$_cd" && pwd -P)
}

current_local() {
    git config --local --get core.hooksPath 2>/dev/null || printf 'unset'
}

# ONE install routine, used by the real install and by --self-test.
do_install() {
    _mode=$1
    _force=$2
    _common=$(common_dir) || { echo "install_hooks: not inside a git repository"; return 1; }
    for f in $HOOK_FILES; do
        [ -f "tools/hooks/$f" ] || { echo "install_hooks: tools/hooks/$f is missing"; return 1; }
    done
    [ -f tools/scripts/leak_scan.py ] || { echo "install_hooks: the scanner is missing"; return 1; }
    _dest=$_common/leak-guard-hooks
    if [ "$_mode" = in-tree ]; then
        _target=tools/hooks
    else
        _target=$_dest
    fi
    _prev=$(current_local)
    _recorded=$(git config --local --get leakguard.previousHooksPath 2>/dev/null || true)
    case "$_prev" in
        unset|"$_dest"|tools/hooks|"$_common/hooks") ;;
        *)
            if [ "$_force" != yes ]; then
                echo "install_hooks: core.hooksPath is already set to a directory this script did not"
                echo "install_hooks: create; re-run with --force to replace it (the hooks chain to the"
                echo "install_hooks: global hook directory and to the repository hooks, not to it)"
                return 1
            fi
            ;;
    esac
    if [ "$_mode" != in-tree ]; then
        mkdir -p "$_dest" || return 1
        for f in $HOOK_FILES; do
            cp "tools/hooks/$f" "$_dest/$f" || return 1
        done
        cp tools/scripts/leak_scan.py "$_dest/leak_scan.py" || return 1
        chmod 755 "$_dest/commit-msg" "$_dest/pre-commit" "$_dest/leak_scan.py" || return 1
    fi
    # Record what was there so --uninstall can put it back; never overwrite an
    # earlier record with our own value.
    case "$_prev" in
        "$_dest"|tools/hooks) [ -n "$_recorded" ] || git config --local leakguard.previousHooksPath unset ;;
        *) git config --local leakguard.previousHooksPath "$_prev" ;;
    esac
    git config --local core.hooksPath "$_target" || return 1
    echo "install_hooks: core.hooksPath = $_target"
    echo "install_hooks: hooks installed (commit-msg, pre-commit); they chain to the previous hooks"
    return 0
}

do_uninstall() {
    _prev=$(git config --local --get leakguard.previousHooksPath 2>/dev/null || true)
    if [ -z "$_prev" ]; then
        echo "install_hooks: nothing recorded to restore"
        return 1
    fi
    if [ "$_prev" = unset ]; then
        git config --local --unset core.hooksPath 2>/dev/null || true
    else
        git config --local core.hooksPath "$_prev"
    fi
    git config --local --unset leakguard.previousHooksPath 2>/dev/null || true
    _common=$(common_dir) || return 1
    rm -rf "$_common/leak-guard-hooks"
    echo "install_hooks: core.hooksPath restored to $_prev"
    return 0
}

do_status() {
    _common=$(common_dir) || { echo "install_hooks: not inside a git repository"; return 1; }
    _local=$(current_local)
    _global=$(git config --global --get core.hooksPath 2>/dev/null || printf 'unset')
    echo "local core.hooksPath:  $_local"
    echo "global core.hooksPath: $_global"
    _dest=$_common/leak-guard-hooks
    _state=inactive
    case "$_local" in
        "$_dest") _state=active-absolute ;;
        tools/hooks) _state=active-in-tree ;;
    esac
    echo "leak guard hooks:      $_state"
    if [ "$_state" = active-absolute ]; then
        _stale=no
        for f in $HOOK_FILES; do
            cmp -s "tools/hooks/$f" "$_dest/$f" || _stale=yes
        done
        cmp -s tools/scripts/leak_scan.py "$_dest/leak_scan.py" || _stale=yes
        echo "installed copy stale:  $_stale"
    fi
    if [ -n "${LEAK_PATTERNS:-}" ]; then
        echo "private patterns:      from the environment"
    elif [ -n "${LEAK_PATTERNS_FILE:-}" ]; then
        echo "private patterns:      from the file named by the environment"
    elif [ -f "${HOME:-/nonexistent}/.config/cerulion/leak-patterns.txt" ]; then
        echo "private patterns:      the default file exists"
    else
        echo "private patterns:      NOT configured (generic classes only)"
    fi
    return 0
}

# --- self-test: a scratch repository with masked global config -------------
self_test() {
    _fails=0
    _arms=0
    fail() {
        echo "SELF-TEST FAILED: $1"
        _fails=$((_fails + 1))
    }
    arm() {
        _arms=$((_arms + 1))
    }
    _src=$(pwd -P)
    _tmp=$(mktemp -d "${TMPDIR:-/tmp}/leak-guard-hooks.XXXXXX") || exit 1
    trap 'rm -rf "$_tmp"' EXIT
    # Mask every configuration outside the scratch repository.
    export HOME=$_tmp/home
    export GIT_CONFIG_GLOBAL=/dev/null
    export GIT_CONFIG_NOSYSTEM=1
    export GIT_AUTHOR_NAME="Self Test"
    export GIT_COMMITTER_NAME="Self Test"
    export GIT_AUTHOR_EMAIL="self-test@users.noreply.github.com"
    export GIT_COMMITTER_EMAIL="self-test@users.noreply.github.com"
    unset LEAK_PATTERNS LEAK_PATTERNS_FILE
    mkdir -p "$HOME"
    _repo=$_tmp/repo
    mkdir -p "$_repo"
    git -C "$_repo" init -q || exit 1
    git -C "$_repo" symbolic-ref HEAD refs/heads/main
    echo "clean" > "$_repo/README.md"
    git -C "$_repo" add README.md
    git -C "$_repo" commit -q -m "first" || exit 1
    _before=$(git -C "$_repo" rev-parse HEAD)
    mkdir -p "$_repo/tools/hooks" "$_repo/tools/scripts"
    for f in $HOOK_FILES; do
        cp "$_src/tools/hooks/$f" "$_repo/tools/hooks/$f"
    done
    cp "$_src/tools/scripts/leak_scan.py" "$_repo/tools/scripts/leak_scan.py"
    cp "$_src/tools/scripts/install_hooks.sh" "$_repo/tools/scripts/install_hooks.sh"
    git -C "$_repo" add -A
    git -C "$_repo" commit -q -m "add the guard" || exit 1

    # A leak that the GENERIC tier catches, assembled so this file stays clean.
    _u=qz
    _u=${_u}rkv
    _leak="/Us""ers/$_u/notes"

    # 1. install (absolute mode) from inside the repository
    arm
    ( cd "$_repo" && ./tools/scripts/install_hooks.sh >/dev/null ) || fail "install returned nonzero"
    _hp=$(git -C "$_repo" config --local --get core.hooksPath || true)
    _common=$(cd "$_repo" && cd "$(git rev-parse --git-common-dir)" && pwd -P)
    arm
    [ "$_hp" = "$_common/leak-guard-hooks" ] || fail "core.hooksPath is not the absolute copy"
    arm
    [ -x "$_common/leak-guard-hooks/pre-commit" ] || fail "installed pre-commit is not executable"

    # 2. a planted staged leak is refused
    echo "see $_leak" > "$_repo/notes.md"
    git -C "$_repo" add notes.md
    arm
    if git -C "$_repo" commit -q -m "docs: notes" >/dev/null 2>&1; then
        fail "a staged leak was committed"
    fi
    git -C "$_repo" reset -q notes.md
    rm -f "$_repo/notes.md"

    # 3. a planted message is refused (the tree is clean)
    echo "more" >> "$_repo/README.md"
    git -C "$_repo" add README.md
    arm
    if git -C "$_repo" commit -q -m "docs: from $_leak" >/dev/null 2>&1; then
        fail "a leaking message was committed"
    fi
    # 3b. the same leak on a line starting with '#' is refused too: git KEEPS such
    #     a line when no editor ran (-m, -F, a script), so the hook must read it
    arm
    if git -C "$_repo" commit -q -m "docs: readme" -m "# from $_leak" >/dev/null 2>&1; then
        fail "a leaking '#' line in a -m message was committed"
    fi
    # 3c. with an editor, git's own template is not the message: an untracked file
    #     whose name is a private pattern must not refuse the commit through the
    #     '# Untracked files:' block, while the same name in the message body must
    _pat=zq
    _pat=${_pat}standin7
    : > "$_repo/notes-$_pat.md"
    _ed=$_tmp/editor.sh
    cat > "$_ed" <<'EOF_EDITOR'
#!/bin/sh
printf 'docs: edited\n' > "$1.new"; cat "$1" >> "$1.new"; mv "$1.new" "$1"
EOF_EDITOR
    chmod +x "$_ed"
    arm
    if ! LEAK_PATTERNS="word:$_pat @host" GIT_EDITOR="$_ed" git -C "$_repo" commit -q >/dev/null 2>&1; then
        fail "an editor commit was refused because of an untracked file name in git's template"
    fi
    echo "again" >> "$_repo/README.md"
    git -C "$_repo" add README.md
    arm
    if LEAK_PATTERNS="word:$_pat @host" git -C "$_repo" commit -q -m "docs: on $_pat" >/dev/null 2>&1; then
        fail "a private name in a -m message was committed"
    fi
    rm -f "$_repo/notes-$_pat.md"

    # 4. a clean commit passes
    arm
    git -C "$_repo" commit -q -m "docs: readme" >/dev/null 2>&1 || fail "a clean commit was refused"

    # 5. a linked worktree on a branch WITHOUT tools/hooks is still covered
    _wt=$_tmp/wt
    git -C "$_repo" worktree add -q -b nohooks "$_wt" "$_before" || exit 1
    arm
    [ ! -d "$_wt/tools/hooks" ] || fail "the worktree fixture unexpectedly carries tools/hooks"
    echo "see $_leak" > "$_wt/w.md"
    git -C "$_wt" add w.md
    arm
    if git -C "$_wt" commit -q -m "docs: w" >/dev/null 2>&1; then
        fail "a leak was committed in a worktree without tools/hooks"
    fi
    git -C "$_wt" reset -q w.md
    echo "clean" > "$_wt/w.md"
    git -C "$_wt" add w.md
    arm
    git -C "$_wt" commit -q -m "docs: w" >/dev/null 2>&1 || fail "a clean worktree commit was refused"

    # 6. a foreign core.hooksPath is refused without --force, replaced with it
    _other=$_tmp/repo2
    mkdir -p "$_other"
    git -C "$_other" init -q
    mkdir -p "$_other/tools/hooks" "$_other/tools/scripts" "$_other/elsewhere"
    for f in $HOOK_FILES; do
        cp "$_src/tools/hooks/$f" "$_other/tools/hooks/$f"
    done
    cp "$_src/tools/scripts/leak_scan.py" "$_other/tools/scripts/leak_scan.py"
    cp "$_src/tools/scripts/install_hooks.sh" "$_other/tools/scripts/install_hooks.sh"
    git -C "$_other" config --local core.hooksPath "$_other/elsewhere"
    arm
    if ( cd "$_other" && ./tools/scripts/install_hooks.sh >/dev/null 2>&1 ); then
        fail "a foreign core.hooksPath was replaced without --force"
    fi
    arm
    ( cd "$_other" && ./tools/scripts/install_hooks.sh --force >/dev/null ) || fail "--force install failed"
    arm
    ( cd "$_other" && ./tools/scripts/install_hooks.sh --uninstall >/dev/null ) || fail "uninstall after --force failed"
    arm
    [ "$(git -C "$_other" config --local --get core.hooksPath)" = "$_other/elsewhere" ] \
        || fail "uninstall did not restore the foreign value"

    # 7. status runs; uninstall restores the unset state
    arm
    ( cd "$_repo" && ./tools/scripts/install_hooks.sh --status >/dev/null ) || fail "status failed"
    arm
    ( cd "$_repo" && ./tools/scripts/install_hooks.sh --uninstall >/dev/null ) || fail "uninstall failed"
    arm
    if git -C "$_repo" config --local --get core.hooksPath >/dev/null 2>&1; then
        fail "uninstall left core.hooksPath set"
    fi
    arm
    [ ! -d "$_common/leak-guard-hooks" ] || fail "uninstall left the installed copy"
    # after uninstall the planted message goes through: proves the refusals above came from the hook
    echo "again" >> "$_repo/README.md"
    git -C "$_repo" add README.md
    arm
    git -C "$_repo" commit -q -m "docs: from $_leak" >/dev/null 2>&1 \
        || fail "after uninstall a commit was still refused (the refusal was not the hook)"

    # 8. in-tree mode sets the relative path
    arm
    ( cd "$_repo" && ./tools/scripts/install_hooks.sh --in-tree >/dev/null ) || fail "in-tree install failed"
    arm
    [ "$(git -C "$_repo" config --local --get core.hooksPath)" = tools/hooks ] || fail "in-tree path not set"
    arm
    ( cd "$_repo" && ./tools/scripts/install_hooks.sh --uninstall >/dev/null ) || fail "in-tree uninstall failed"

    _expected=24
    if [ "$_arms" -ne "$_expected" ]; then
        fail "arm count $_arms differs from the pinned $_expected"
    fi
    if [ "$_fails" -ne 0 ]; then
        exit 1
    fi
    echo "install_hooks --self-test: OK (arms=$_arms, bash ${BASH_VERSION%%(*})"
    exit 0
}

case "$MODE" in
    install) do_install absolute "$FORCE" || exit 1 ;;
    in-tree) do_install in-tree "$FORCE" || exit 1 ;;
    uninstall) do_uninstall || exit 1 ;;
    status) do_status || exit 1 ;;
    self-test) self_test ;;
esac
exit 0
