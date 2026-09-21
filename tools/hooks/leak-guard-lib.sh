#!/bin/sh
# Shared by the leak guard hooks in this directory. Sourced, never executed.
#
# Every check lives in tools/scripts/leak_scan.py; a hook resolves the scanner,
# runs ONE hook mode, then CHAINS to the hook that was active before the guard
# was installed (the global core.hooksPath directory when one exists, else the
# repository's own hooks directory), so nothing that used to run stops running.
#
# A hook must not brick a contributor's commit: a missing scanner or a missing
# python3 prints one line and lets the commit through (CI is the floor). A HARD
# hit blocks; the scanner names the fix and the bypass.

leak_guard_run() {
    lg_hook=$1
    shift
    lg_dir=$(cd "$(dirname "$0")" && pwd -P)
    lg_top=$(git rev-parse --show-toplevel 2>/dev/null) || lg_top=
    lg_scanner=
    if [ -n "$lg_top" ] && [ -f "$lg_top/tools/scripts/leak_scan.py" ]; then
        lg_scanner=$lg_top/tools/scripts/leak_scan.py
    elif [ -f "$lg_dir/leak_scan.py" ]; then
        lg_scanner=$lg_dir/leak_scan.py
    fi
    if [ -z "$lg_scanner" ]; then
        echo "leak-guard: scanner not found; the $lg_hook check was skipped" >&2
    elif ! command -v python3 >/dev/null 2>&1; then
        echo "leak-guard: python3 not found; the $lg_hook check was skipped" >&2
    else
        # An installed copy compares itself to the tree's copy: the tree moves, the copy does not.
        if [ -n "$lg_top" ] && [ -f "$lg_top/tools/hooks/$lg_hook" ] \
            && [ "$lg_dir" != "$(cd "$lg_top/tools/hooks" && pwd -P)" ] \
            && ! cmp -s "$0" "$lg_top/tools/hooks/$lg_hook"; then
            echo "leak-guard hooks are stale, re-run tools/scripts/install_hooks.sh" >&2
        fi
        python3 -B "$lg_scanner" hook "$lg_hook" "$@" || exit $?
    fi
    leak_guard_chain "$lg_hook" "$lg_dir" "$@"
}

leak_guard_chain() {
    lg_hook=$1
    lg_dir=$2
    shift 2
    lg_global=$(git config --global --type=path --get core.hooksPath 2>/dev/null) \
        || lg_global=$(git config --global --get core.hooksPath 2>/dev/null) || lg_global=
    # shellcheck disable=SC2088  # a LITERAL tilde is what an unexpanded config value carries
    case "$lg_global" in
        "~/"*) lg_global=$HOME/${lg_global#"~/"} ;;
    esac
    lg_common=$(git rev-parse --git-common-dir 2>/dev/null) || lg_common=
    [ -n "$lg_common" ] && lg_common=$(cd "$lg_common" && pwd -P)
    lg_next=
    if [ -n "$lg_global" ] && [ -x "$lg_global/$lg_hook" ] \
        && [ "$(cd "$lg_global" && pwd -P)" != "$lg_dir" ]; then
        lg_next=$lg_global/$lg_hook
    elif [ -n "$lg_common" ] && [ -x "$lg_common/hooks/$lg_hook" ] \
        && [ "$lg_common/hooks" != "$lg_dir" ]; then
        lg_next=$lg_common/hooks/$lg_hook
    fi
    if [ -n "$lg_next" ]; then
        exec "$lg_next" "$@"
    fi
    exit 0
}
