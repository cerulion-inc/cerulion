# shellcheck shell=bash
# Workspace-hardening helpers for run_matrix.sh (they stop a hostile
# workspace from executing hooks). Sourced, never executed: run_matrix.sh
# uses these inside the container, and run_matrix_selftest.sh proves them
# DESK-SIDE (bash + git only) — including that the git actually running
# them honors the config channels the wrapper forces off.

# dev:inode of the object a path RESOLVES to (GNU stat, then BSD). -L
# is deliberate: the identity of what git will actually operate on, and the
# only reading under which a Linux /dev/fd/N magic link reports the
# OPENED directory (fstat semantics) rather than the proc symlink itself.
# NOT for the mutant registry: `ws_file_identity` below is its no-follow
# sibling, and following is the one thing that registry must never do.
ws_identity() {
    stat -L -c '%d:%i' "$1" 2>/dev/null || stat -L -f '%d:%i' "$1"
}

# EVERY git invocation the harness itself runs inside the
# scratch workspace goes through this wrapper. A post-bind replacement of
# the workspace can plant .git/hooks/* and .git/config; command-line
# config is git's HIGHEST-precedence layer, so forcing the
# code-execution channels off here beats anything a replaced tree
# carries: a hooksPath aimed at a non-directory disables every hook
# (pre-commit, post-checkout, ...), and fsmonitor=false stops a
# config-named monitor command from being exec'd by ANY git operation
# (boolean fsmonitor needs git >= 2.36; the container's 2.43 and the
# selftest's local git both honor it — the selftest proves acceptance
# empirically). GIT_CONFIG_GLOBAL/SYSTEM=/dev/null shield the harness
# from ambient per-user/system config for the same reason. Scope: the
# HARNESS's own git only — stage 6's `cerulion ros2 migrate --write`
# deliberately keeps the ENGINE's user-hook semantics (a user's
# pre-commit legitimately gates the migration commit; the engine e2e
# suite depends on exactly that).
#
# Clean/smudge filters: filters have no
# hooksPath-style kill switch and their names are attacker-chosen, so
# `-c` cannot enumerate them away. What closes the channel is git's own
# precedence: attributes (in-tree .gitattributes included) only NAME a
# filter — the COMMAND resolves exclusively from config
# (filter.<name>.smudge/clean/process), and a named filter with no
# configured command is documented PASSTHROUGH (required=true would
# error, not execute, and itself needs config). ws_git nulls the system
# + global config layers here, and the remaining command sources —
# .git/config, .git/config.worktree, .git/info/attributes — all live in
# the GITDIR, which run_matrix.sh identity-binds right after its own
# `ws_git init` and re-verifies (require_ws_and_gitdir) before every
# executing stage. GIT_ATTR_NOSYSTEM=1 is the system-attributes belt.
# (`git apply` never runs smudge/clean — it writes worktree bytes
# directly — and `--check` writes nothing; it is wrapped anyway.)
#
# Two further channels are closed here:
#
# Runtime config injection: shielding config files does not shield the
# RUNTIME config-injection env. GIT_CONFIG_COUNT + GIT_CONFIG_KEY_n/VALUE_n
# survive into the child and inject filter.<name>.smudge/clean at command-line
# precedence — above everything the wrapper forces. Rather than
# enumerate that one family, EVERY ambient GIT_* variable is cleared
# (`env -u` over the live environment), which also closes the
# repo-redirection family (GIT_DIR, GIT_WORK_TREE, GIT_INDEX_FILE,
# GIT_OBJECT_DIRECTORY, ...) and the exec-y family (GIT_SSH_COMMAND,
# GIT_ASKPASS, GIT_EDITOR, GIT_EXTERNAL_DIFF, ...) in one sweep. The
# ALLOWLIST is exactly the wrapper's own three assignments below —
# nothing ambient survives. (A GIT_-prefixed name with characters
# outside [A-Za-z0-9_] cannot be a git-honored variable; a multi-line
# value can at worst add a spurious `-u NAME` for a name that does not
# exist, which `env -u` ignores.)
#
# The gate lives in the wrapper: an identity gate that sits only at stage
# boundaries leaves the harness's own add/commit/checkout, between GITDIR_ID
# capture and the next boundary, UNGATED. The gate is folded INTO ws_git —
# un-skippable: run_matrix publishes the bound identities in
# WS_BIND_PATH / WS_BIND_ID / WS_BIND_GITDIR_ID, and every invocation
# re-verifies them before exec'ing git (the workspace once its id is
# bound; the gitdir too once ITS id is bound — the one pre-gitdir-bind
# caller is the harness's own `init`, which is what MINTS the gitdir).
# The standalone require_* helpers remain for the non-git executing
# stages (colcon, matrix_runner). Cost: a stat pair per git call.
#
# The gate itself needs two more properties:
#
# Bind-variable ownership: the harness OWNS its bind variables. A
# WS_BIND_GITDIR_ID inherited from the caller's environment would reach the
# harness's first `ws_git init` before the harness has minted a gitdir, so the
# wrapper would gate the not-yet-existing .git against a stale value and the
# matrix would exit 2 before setup. ws_bind_publish — the ONE seam where a run
# takes ownership — first DISOWNS every ambient WS_BIND_*: named loudly,
# ignored, and `unset` rather than overwritten, because an assignment
# keeps an inherited variable's EXPORT attribute and would leak the
# run's own bindings into every child process's environment. The gitdir
# identity is published only after the run's own init (ws_bind_gitdir).
#
# Object binding: a gate that verifies the workspace and gitdir by
# PATHNAME and then starts git by PATHNAME (`-C "$WS"`) is racy: a same-user
# writer swapping the path between the check and the exec runs git in a
# replacement repository whose LOCAL config supplies a smudge command
# (reproduced with a pause at exactly that instant). The wrapper
# therefore BINDS git to the objects it verified, so check and exec share
# one inode:
#   - the WORKTREE half is portable: ws_git enters the workspace in a
#     subshell (`cd`), verifies the identity of `.` — the directory
#     object its cwd is now bound to — and execs git with NO -C, so git
#     inherits that cwd. A pathname swap after the cd changes what the
#     name points at, not where this process is (the same object
#     binding run_matrix.sh's cleanup trap already uses).
#   - the GITDIR half is fd-bound where the host allows it (Linux — the
#     container): `.git` is OPENED relative to the bound cwd, its
#     identity is read off the OPEN descriptor (`stat -L /dev/fd/N` is
#     fstat of the opened object — measured identical to the pathname's
#     dev:inode on Linux; without -L it is the proc symlink), and git is
#     pointed at the descriptor (`--git-dir=/dev/fd/N`), so every gitdir
#     access git makes resolves through the open file and a `.git`
#     swapped inside the workspace after the open is never consulted.
#     MEASURED on Linux (git 2.34; the container's 2.43 runs the same
#     explicit-gitdir setup path): rev-parse/status/add/commit/checkout
#     all work through the descriptor (index.lock rename included), and
#     after either swap — whole workspace, or `.git` alone — the
#     fd-bound checkout restored the ORIGINAL object's file with no
#     filter fired, while the pathname control landed in the hostile
#     tree and fired its smudge. bash `{fd}` descriptors are inherited
#     by `stat`, `env -u` and `git`, so all three see one object.
#     macOS CANNOT do this: devfs `/dev/fd/N` is not traversable —
#     `cd /dev/fd/N`, `git -C /dev/fd/N` and `--git-dir=/dev/fd/N` all
#     fail ("Not a directory" / "not a git repository"; measured) — so
#     there ws_git resolves `.git` BY NAME inside the cwd-bound
#     workspace and the gitdir check→exec instant stays open on the
#     desk, where the harness never runs. run_matrix.sh REFUSES to start
#     without fd binding (ws_git_require_fd_binding), so the production
#     wrapper can never silently run in the by-name mode; the selftest
#     proves the by-name half on macOS and skips its fd-only arm loudly.
#   - the git-facing argv is CONSTRAINED: leading `-c KEY=VALUE` pairs,
#     then a subcommand from a fixed allowlist of the builtins the
#     harness uses, then that subcommand's arguments. `-C`, `--git-dir`,
#     `--work-tree`, `--exec-path` and every other global option are
#     refused (each re-targets git by pathname or by executable); a
#     repository-defined ALIAS can never be selected (git never consults
#     aliases for builtin names, and non-builtin names are refused);
#     `--no-pager` closes the pager exec channel; and the wrapper's own
#     `-c` overrides come AFTER the caller's, so they win.
# Known residuals: (a) git's own startup re-derives the
# worktree as an absolute pathname (setup_work_tree chdir()s to the
# realpath of the cwd it inherited) — a sub-millisecond window inside
# git's process, not a shell-level seam, and one in which the gitdir
# stays fd-bound, so every execution channel is still config-gated: a
# replacement WORKTREE alone carries no executable channel through
# these builtins (hooks off, fsmonitor off, filters need config, no
# pager); (b) in-place edits inside the bound gitdir — the known
# residual, unchanged; (c) on hosts without fd binding, (b) widens to a
# by-name swap of `.git` inside the bound workspace.
#
# The harness also guarantees the following:
#
# ws_git sweeps the ambient GIT_* surface for ITS OWN git only; the
# harness also launches the ENGINE's git — stage 6's real `cerulion ros2
# migrate`, which would inherit the harness environment verbatim, so an
# ambient GIT_DIR / GIT_WORK_TREE / GIT_INDEX_FILE would point the
# migration commit at an EXTERNAL repository. The sweep is factored out
# as git_env_sweep and every verb launch goes through it (nothing else
# forced there — the engine keeps the user's hooks + config, and it
# supplies its own identity fallback, reading none from this
# environment). The fd-binding verdict cache never trusts an inherited
# WS_GIT_FD_BINDING: it is stamped with the probing shell's pid, so
# a fresh shell always probes. The selftest's disown oracle names
# all three ambient variables, not one.
#
# The verb launch is bound the same way:
#
# Verifying the workspace + gitdir and then handing the
# ENGINE the still-replaceable pathname (`--workspace "$WS"`) is the
# CLI-launch twin of the ws_git pathname seam, with the engine honoring
# hooks BY DESIGN (reproduced: the replacement's pre-commit hook fires). The
# verb therefore launches through ws_bound_launch — the SAME object binding
# ws_git uses (cd into the workspace, verify `.`, verify the gitdir
# through its descriptor on Linux, then exec) — with `--workspace .`, so
# the engine starts INSIDE the object the check saw. Known residual:
# the engine canonicalize()s `.` to a pathname at startup and addresses
# the workspace by that pathname for its whole run (analysis, then
# add/commit) — an ENGINE property the harness cannot bind; a swap in
# that window is DETECTED by stage 6's HEAD-advanced / clean-tree
# oracle (the bound object did not get the commit), never reported as
# success. A verb-launch sweep that also stripped GIT_CONFIG_* and the
# identity variables would stop the e2e driving the CLI the way a
# user's shell does; git_env_sweep --keep-user-config therefore KEEPS the
# user's config + identity families (the stated keep-list below) and
# strips only the redirection + exec families — ws_git keeps the full
# sweep. The selftest's fd-binding oracle is independent of the
# function under test.
ws_git() {
    if [ -z "${WS_BIND_PATH:-}" ]; then
        echo "error: ws_git invoked with no published workspace binding \
(ws_bind_publish) — the wrapper only ever runs git inside the bound \
workspace" >&2
        return 2
    fi
    local cfg=() sub=
    while [ $# -gt 0 ]; do
        case "$1" in
            -c)
                if [ $# -lt 2 ]; then
                    echo "error: ws_git: -c needs a KEY=VALUE" >&2
                    return 2
                fi
                cfg+=(-c "$2")
                shift 2
                ;;
            -*)
                echo "error: ws_git: refusing global git option '$1' — the \
wrapper accepts leading -c KEY=VALUE pairs followed by a subcommand; -C, \
--git-dir, --work-tree, --exec-path and friends re-target git by pathname \
or by executable and would defeat the object binding" >&2
                return 2
                ;;
            *)
                sub=$1
                shift
                break
                ;;
        esac
    done
    case "$sub" in
        init | add | commit | checkout | status | apply | rev-parse | log) ;;
        *)
            echo "error: ws_git: refusing subcommand '${sub:-<none>}' — the \
wrapper runs only the builtins the harness uses (init add commit checkout \
status apply rev-parse log); a repository-defined alias can never be selected" >&2
            return 2
            ;;
    esac
    # Probe + cache the fd-binding verdict in THIS shell so the subshell
    # below inherits it instead of re-probing on every call.
    ws_git_fd_binding_available || true
    (
        ws_enter_bound_workspace "this ws_git invocation" || exit 2
        local gitdir=()
        if [ -n "${WS_BIND_GITDIR_ID:-}" ]; then
            ws_open_bound_gitdir "this ws_git invocation" || exit 2
            if [ -n "$WS_BOUND_GFD" ]; then
                gitdir=(--git-dir="/dev/fd/$WS_BOUND_GFD")
            else
                gitdir=(--git-dir=.git)
            fi
        fi
        ws_pause_seam
        git_env_sweep \
            GIT_CONFIG_GLOBAL=/dev/null GIT_CONFIG_SYSTEM=/dev/null GIT_ATTR_NOSYSTEM=1 \
            git "${cfg[@]}" --no-pager \
            -c core.hooksPath=/dev/null -c core.fsmonitor=false \
            "${gitdir[@]}" "$sub" "$@"
    )
}

# The object-binding halves ws_git and ws_bound_launch share. Both run
# INSIDE the caller's subshell (they cd and open descriptors there).
#
# Enter the bound workspace and verify the OBJECT the cwd is now bound
# to — a pathname swap after the cd changes what the name points at, not
# where this process is.
ws_enter_bound_workspace() {
    # $1 label.
    cd "$WS_BIND_PATH" 2>/dev/null || {
        echo "error: cannot enter bound workspace '$WS_BIND_PATH' for $1" >&2
        return 2
    }
    if [ -n "${WS_BIND_ID:-}" ]; then
        require_ws_identity . "$WS_BIND_ID" \
            "$1 (cwd-bound workspace '$WS_BIND_PATH')" || return 2
    fi
}
# What kind of object a name currently is (for refusal messages).
ws_object_kind() {
    if [ -L "$1" ]; then echo symlink
    elif [ -d "$1" ]; then echo directory
    elif [ -p "$1" ]; then echo FIFO
    elif [ -S "$1" ]; then echo socket
    elif [ -f "$1" ]; then echo "regular file"
    elif [ -e "$1" ]; then echo "other object"
    else echo missing
    fi
}
# Open the gitdir INSIDE the entered workspace (this closes two gaps:
# the FIFO hang and the pathname-derived bind). The TYPE is
# gated BEFORE any open: `.git` must be a real directory — never a
# symlink (a redirect), a FIFO (a writerless one blocks a plain open
# forever, wedging every ws_git and every bound launch), a socket, a
# device or a file. Refused without reading or waiting. fd hosts then
# open it relative to the bound cwd, fstat the OPENED descriptor to be a
# directory as well, and hand it back in WS_BOUND_GFD (git is pointed at
# /dev/fd/N; the bind derives WS_BIND_GITDIR_ID from this same
# descriptor). The open itself runs under a watchdog for the residual
# swap between the gate and the open — a FIFO landing in that instant
# turns into a bounded, loud failure instead of a hang. Hosts without fd
# binding open nothing (WS_BOUND_GFD empty). The brace group scopes the
# stderr redirect to the open attempt (a bare `exec {fd}<x 2>/dev/null`
# reports the failure before the redirect applies) while the descriptor
# persists in the subshell.
ws_open_gitdir_in_cwd() {
    # $1 label. Sets WS_BOUND_GFD.
    WS_BOUND_GFD=
    if [ -L .git ] || [ ! -d .git ]; then
        echo "error: '.git' inside the bound workspace is not a real \
directory (it is a $(ws_object_kind .git)) before $1 — refusing to open it" >&2
        return 2
    fi
    ws_git_fd_binding_available || return 0
    local gfd='' me=$BASHPID watchdog
    (
        sleep "${WS_GIT_OPEN_TIMEOUT:-5}"
        kill -TERM "$me" 2>/dev/null
    ) >/dev/null 2>&1 &
    watchdog=$!
    { exec {gfd}<.git; } 2>/dev/null || true
    kill "$watchdog" 2>/dev/null
    wait "$watchdog" 2>/dev/null || true
    if [ -z "$gfd" ] || [ ! -d "/dev/fd/$gfd" ]; then
        if [ -n "$gfd" ]; then
            exec {gfd}<&-
        fi
        echo "error: '.git' inside the bound workspace could not be opened as \
a directory before $1 — refusing" >&2
        return 2
    fi
    WS_BOUND_GFD=$gfd
}
# Open + VERIFY the gitdir against the published identity. fd hosts:
# identity read off the OPEN descriptor; other hosts: verified by name.
ws_open_bound_gitdir() {
    # $1 label. Sets WS_BOUND_GFD.
    ws_open_gitdir_in_cwd "$1" || return 2
    if [ -n "$WS_BOUND_GFD" ]; then
        require_ws_identity "/dev/fd/$WS_BOUND_GFD" "$WS_BIND_GITDIR_ID" \
            "$1 (fd-bound gitdir)" || return 2
    else
        require_ws_identity .git "$WS_BIND_GITDIR_ID" \
            "$1 (gitdir by name — no fd binding on this host)" || return 2
    fi
}
# Test seam (verification only): parks a bound wrapper at the exact
# bind/verify -> exec instant so the selftest can swap the pathname (or
# .git) deterministically and prove the exec stays on the verified
# objects. Two FIFOs (ready: wrapper -> tester, go: tester -> wrapper) so
# a line can never be read back by its own writer. Inert unless set.
ws_pause_seam() {
    if [ -n "${CERULION_WS_PAUSE_DIR:-}" ]; then
        echo paused > "$CERULION_WS_PAUSE_DIR/ready"
        read -r _ < "$CERULION_WS_PAUSE_DIR/go" || true
    fi
}

# Launch a non-git command — the ENGINE — inside
# the bound workspace object: the same cd-binding + identity checks as
# ws_git, then the user-config-preserving sweep. Stage 6 addresses the
# workspace as `--workspace .`, so the engine starts in the verified
# object rather than at a pathname a same-user writer can swap after the
# check.
#
# Gitdir TOCTOU: the verified gitdir
# descriptor is kept OPEN across the launch (it is never closed before the
# exec). Stated at its true strength: an open descriptor pins the verified
# gitdir OBJECT for the engine's lifetime (it cannot be reclaimed or its
# inode recycled while the engine runs, and it is what the post-launch
# oracle addresses), but it does NOT stop a rename swap at the `.git`
# NAME — and the engine resolves `.git` by name at startup (it
# canonicalize()s `--workspace .` and addresses the repository by
# pathname), so a swap in the instant between this verify and the
# engine's own open is not prevented by anything the launcher holds. What
# would close that instant is the engine consuming a handed descriptor for
# its gitdir — an engine-side, Linux-only change (git accepts
# `--git-dir=/proc/self/fd/N`; macOS cannot) that the engine does not
# implement. The guarantee is therefore
# DETECTION: stage 6's HEAD-advanced / clean-tree oracle reads the bound
# object through ws_git and fails the stage if the engine's commit landed
# anywhere else. The engine's own git children run under an explicit
# GIT_* keep-list policy as well, so an ambient redirection cannot
# point them elsewhere.
ws_bound_launch() {
    if [ -z "${WS_BIND_PATH:-}" ]; then
        echo "error: ws_bound_launch invoked with no published workspace \
binding (ws_bind_publish)" >&2
        return 2
    fi
    ws_git_fd_binding_available || true
    (
        ws_enter_bound_workspace "this bound launch" || exit 2
        if [ -n "${WS_BIND_GITDIR_ID:-}" ]; then
            # Held open across the launch — see above.
            ws_open_bound_gitdir "this bound launch" || exit 2
        fi
        ws_pause_seam
        git_env_sweep --keep-user-config "$@"
    )
}

# Run COMMAND with the ambient GIT_* surface removed
# (`env -u` over the live environment — the ws_git sweep), factored
# out of ws_git because the harness ALSO launches the ENGINE (stage 6's
# `cerulion ros2 migrate`), which would otherwise inherit the harness
# environment verbatim. Leading NAME=VALUE arguments are honored by env(1), which
# is how ws_git passes its own three assignments through.
#
# Two sweeps, stated explicitly.
#   - default (ws_git): EVERY ambient GIT_* is removed — the wrapper
#     then forces its own config layers, so nothing of the user's
#     environment is meant to reach the harness's own git.
#   - --keep-user-config (the ENGINE launch): the e2e must drive the CLI
#     the way a user's shell would, so the KEEP-LIST below passes
#     through — the config FILE selectors (GIT_CONFIG_GLOBAL / SYSTEM /
#     NOSYSTEM), the runtime config injection (GIT_CONFIG_COUNT /
#     KEY_n / VALUE_n, GIT_CONFIG_PARAMETERS), attributes
#     (GIT_ATTR_NOSYSTEM) and identity (GIT_AUTHOR_* / GIT_COMMITTER_*)
#     — while everything else is stripped: the REDIRECTION family
#     (GIT_DIR, GIT_WORK_TREE, GIT_INDEX_FILE, GIT_OBJECT_DIRECTORY,
#     GIT_ALTERNATE_OBJECT_DIRECTORIES, GIT_COMMON_DIR, GIT_NAMESPACE,
#     GIT_CEILING_DIRECTORIES, GIT_DISCOVERY_ACROSS_FILESYSTEM,
#     GIT_IMPLICIT_WORK_TREE), the EXEC family (GIT_EXEC_PATH,
#     GIT_TEMPLATE_DIR, GIT_SSH, GIT_SSH_COMMAND, GIT_ASKPASS,
#     GIT_EDITOR, GIT_SEQUENCE_EDITOR, GIT_PAGER, GIT_EXTERNAL_DIFF,
#     GIT_PROXY_COMMAND, ...) and any GIT_* a future git adds — it is a
#     KEEP list, so an unknown variable defaults to stripped. The kept
#     set is exactly what shapes WHICH config and identity git uses;
#     the stripped families re-target WHERE git operates or WHAT it
#     executes.
git_env_sweep() {
    local keep_user_config=0
    if [ "${1:-}" = "--keep-user-config" ]; then
        keep_user_config=1
        shift
    fi
    local unset_args=() name
    while IFS= read -r name; do
        if [ "$keep_user_config" = 1 ]; then
            case "$name" in
                GIT_CONFIG_GLOBAL | GIT_CONFIG_SYSTEM | GIT_CONFIG_NOSYSTEM | \
                    GIT_CONFIG_COUNT | GIT_CONFIG_KEY_* | GIT_CONFIG_VALUE_* | \
                    GIT_CONFIG_PARAMETERS | GIT_ATTR_NOSYSTEM | \
                    GIT_AUTHOR_NAME | GIT_AUTHOR_EMAIL | GIT_AUTHOR_DATE | \
                    GIT_COMMITTER_NAME | GIT_COMMITTER_EMAIL | GIT_COMMITTER_DATE)
                    continue
                    ;;
            esac
        fi
        unset_args+=(-u "$name")
    done < <(env | sed -n 's/^\(GIT_[A-Za-z0-9_]*\)=.*/\1/p' | sort -u)
    env "${unset_args[@]}" "$@"
}

# Can git be pointed at an OPENED directory through
# /dev/fd/N? Linux (procfs magic links): yes. macOS (devfs): no — the
# entry is not traversable. Probed once per shell (opening `/` and
# testing traversal), then cached — STAMPED WITH THIS SHELL'S PID ($$,
# the same value in its subshells and background jobs), so a value
# inherited from the environment is never trusted (a trusted
# ambient WS_GIT_FD_BINDING=0 makes the up-front requirement refuse on
# Linux; a trusted ambient =1 sends ws_git down the fd path on a
# host that cannot take it). A fresh shell therefore always PROBES.
ws_git_fd_binding_available() {
    case "${WS_GIT_FD_BINDING:-}" in
        "$$:0" | "$$:1") ;;
        *)
            local pfd='' verdict=0
            if { exec {pfd}</; } 2>/dev/null && [ -e "/dev/fd/$pfd/." ]; then
                verdict=1
            fi
            if [ -n "$pfd" ]; then
                exec {pfd}<&-
            fi
            WS_GIT_FD_BINDING="$$:$verdict"
            ;;
    esac
    [ "$WS_GIT_FD_BINDING" = "$$:1" ]
}

# The harness's up-front requirement: it only ever runs in the Linux
# container, and a host that cannot bind the gitdir must not silently
# run the by-name mode. Probes fresh by construction (the cache is
# pid-stamped — an inherited verdict cannot satisfy it).
ws_git_require_fd_binding() {
    ws_git_fd_binding_available && return 0
    echo "error: this host cannot bind git to an opened directory (/dev/fd/N \
is not traversable here — Linux procfs is required); run_matrix.sh runs \
inside the Linux container (README), where ws_git binds every git call to \
the verified workspace + gitdir objects — refusing to run with a \
pathname-resolved gitdir" >&2
    return 2
}

# Every WS_BIND_* the harness finds already set is
# ambient — named, ignored and `unset` (unset also drops an inherited
# export attribute, which a plain assignment would keep and pass on to
# every child process).
ws_bind_disown_ambient() {
    local v
    for v in WS_BIND_PATH WS_BIND_ID WS_BIND_GITDIR_ID; do
        if [ -n "${!v+x}" ]; then
            echo "warning: ignoring ambient $v='${!v}' inherited from the \
environment — the harness owns its bind variables and publishes its own \
(the gitdir identity only after its own init)" >&2
        fi
    done
    unset WS_BIND_PATH WS_BIND_ID WS_BIND_GITDIR_ID
}

# Publish THIS run's workspace binding — the one seam where a run takes
# ownership: ambient bindings are disowned first, so nothing inherited
# can reach the first ws_git.
ws_bind_publish() {
    # $1 workspace path, $2 its dev:inode as read off the BOUND object by
    # the caller (run_matrix.sh's bind_fresh_ws).
    ws_bind_disown_ambient
    WS_BIND_PATH=$1
    WS_BIND_ID=$2
}

# Capture + publish the gitdir identity from INSIDE the bound workspace
# object right after the run's own `ws_git init`.
# The identity is read off the DESCRIPTOR the type-gated open
# returns (ws_open_gitdir_in_cwd) — the object git will be pointed at —
# never from a separate pathname stat, so what is published and what is
# later used are one resolution, not two. Hosts without fd binding read
# `.git` by name inside the bound cwd (their documented by-name tier).
ws_bind_gitdir() {
    if [ -z "${WS_BIND_PATH:-}" ] || [ -z "${WS_BIND_ID:-}" ]; then
        echo "error: ws_bind_gitdir: no workspace binding published" >&2
        return 2
    fi
    local id
    id=$(
        cd "$WS_BIND_PATH" 2>/dev/null || {
            echo "error: cannot enter bound workspace '$WS_BIND_PATH' for \
the gitdir bind" >&2
            exit 2
        }
        require_ws_identity . "$WS_BIND_ID" \
            "the gitdir bind (cwd-bound workspace '$WS_BIND_PATH')" || exit 2
        ws_open_gitdir_in_cwd "the gitdir bind" || exit 2
        if [ -n "$WS_BOUND_GFD" ]; then
            ws_identity "/dev/fd/$WS_BOUND_GFD"
        else
            ws_identity .git
        fi
    ) || return 2
    WS_BIND_GITDIR_ID=$id
}

# The analysis inputs — the descendant
# directories the prover is pointed at (`src`, the compile-database
# directory) — are bound too. ws_bind_descendant captures a descendant's
# identity from INSIDE the bound workspace (cwd-bound, the descendant
# gated to be a real directory, never a symlink); require_ws_descendant
# re-verifies it immediately before every analyzer execution. The prover
# takes PATHNAMES, so the check→exec instant is the same documented
# residual as colcon's; a swap after the bind is refused, not analyzed.
ws_bind_descendant() {
    # $1 descendant path relative to the bound workspace. Echoes dev:inode.
    if [ -z "${WS_BIND_PATH:-}" ] || [ -z "${WS_BIND_ID:-}" ]; then
        echo "error: ws_bind_descendant: no workspace binding published" >&2
        return 2
    fi
    (
        cd "$WS_BIND_PATH" 2>/dev/null || {
            echo "error: cannot enter bound workspace '$WS_BIND_PATH' to \
bind '$1'" >&2
            exit 2
        }
        require_ws_identity . "$WS_BIND_ID" \
            "the bind of '$1' (cwd-bound workspace '$WS_BIND_PATH')" || exit 2
        # EVERY component down to the descendant is a real
        # directory, not only the last one.
        require_ws_real_dir_chain . "$1" "the bind of '$1'" || exit 2
        ws_identity "$1"
    )
}

# Path traversal: a pathname check that
# lstat-gates only the FINAL component follows a symlinked INTERMEDIATE
# one — move the bound tree aside, link `build` to it, and
# `build/migrate_fixture_pkg` still carries the bound identity while the
# pathname resolves through an actor-controlled link that can be
# retargeted before the prover reads. Every component from the base down
# to the descendant must be a real directory (never a symlink, never a
# non-directory, never `..`) BEFORE the final identity compare.
require_ws_real_dir_chain() {
    # $1 base (already verified), $2 relative path, $3 stage label.
    local acc=$1 comp comps
    IFS='/' read -r -a comps <<< "$2"
    for comp in "${comps[@]}"; do
        if [ -z "$comp" ] || [ "$comp" = "." ]; then
            continue
        fi
        if [ "$comp" = ".." ]; then
            echo "error: '$2' climbs out of the bound workspace ('..') before \
$3 — refusing" >&2
            return 2
        fi
        acc="$acc/$comp"
        if [ -L "$acc" ] || [ ! -d "$acc" ]; then
            echo "error: '$acc' (component '$comp' of '$2') is not a real \
directory (it is a $(ws_object_kind "$acc")) before $3 — a symlinked or \
non-directory component is refused before any identity comparison" >&2
            return 2
        fi
    done
}
require_ws_descendant() {
    # $1 descendant path relative to the bound workspace, $2 its bound
    # dev:inode, $3 stage label. The whole component chain is
    # gated, then the identity compared.
    require_ws_real_dir_chain "${WS_BIND_PATH:-}" "$1" "$3 ($1)" || return 2
    require_ws_identity "${WS_BIND_PATH:-}/$1" "$2" "$3 ($1)" || return 2
}

# A pathname identity check must refuse a
# symlink BEFORE comparing identities — `stat -L` follows a link, so a
# bound directory MOVED aside and its pathname replaced by a symlink to the
# same inode carries the bound identity while the pathname resolves
# through an actor-controlled link (re-pointable at any later instant).
# lstat-aware: the pathname must be a real directory. Used for every
# pathname-addressed check (the workspace, its gitdir, the analysis
# descendants); the raw require_ws_identity stays for the cwd-bound `.`
# and for /dev/fd/N (a magic link by construction on Linux).
require_ws_path_identity() {
    # $1 pathname, $2 expected dev:inode, $3 stage label.
    if [ -L "$1" ] || [ ! -d "$1" ]; then
        echo "error: '$1' is not a real directory (it is a $(ws_object_kind "$1")) \
before $3 — a pathname replaced by a link or a non-directory is refused before \
any identity comparison" >&2
        return 2
    fi
    require_ws_identity "$1" "$2" "$3" || return 2
}

# The prover reads files by pathname — replacing an
# analyzed source or compile_commands.json changes nothing about the
# parent directory's identity, and the check→exec instant of a pathname
# consumer cannot be closed from the shell (clang's file access is
# pathname-based). So the analysis INPUTS are also bound by CONTENT: a
# digest over every regular file (path + sha256) and symlink (path +
# target) under `src`, plus compile_commands.json, captured at bind time
# and re-verified after each analysis stage — a substitution that lands
# anywhere in a stage is DETECTED and fails the matrix instead of
# reporting a verdict over attacker-controlled inputs. Residual, stated: a
# substitution reverted before the post-stage check.
ws_sha256() {
    if command -v sha256sum >/dev/null 2>&1; then
        sha256sum | cut -d' ' -f1
    else
        shasum -a 256 | cut -d' ' -f1
    fi
}
# Reliability: a file read for a digest is
# TYPE-GATED (a real regular file — never a symlink, FIFO, socket, device
# or directory: a writerless FIFO at compile_commands.json would block the
# matrix forever) and the read itself is BOUNDED by the same watchdog as
# the gitdir open, so a FIFO swapped in between the gate and the open is
# a loud failure, not a hang. Echoes the sha256.
ws_file_sha256() {
    # $1 file.
    if [ -L "$1" ] || [ ! -f "$1" ]; then
        echo "error: '$1' is not a regular file (it is a $(ws_object_kind "$1")) \
— refusing to read it" >&2
        return 2
    fi
    local out
    # Robustness, measured: the watchdog is
    # backgrounded INSIDE a command substitution, so without the redirect it
    # inherits the substitution's pipe as its stdout — and `kill "$w"` reaps
    # the SUBSHELL while its `sleep` child, a separate process holding the
    # same write end, is orphaned and lives out the full timeout. The
    # substitution therefore sees no EOF until the bound expires: measured
    # 5.15s per digest (15.46s for three) against 0.129s with the redirect,
    # which is minutes-to-timeout across a tree digest. Redirecting the
    # watchdog's stdout is what closes it; the bound itself is untouched (a
    # writerless FIFO still refuses within WS_GIT_OPEN_TIMEOUT). The same
    # redirect is applied at every watchdog site — the other two are not in a
    # command substitution today, and one call away from being.
    if ! out=$(
        me=$BASHPID
        (
            sleep "${WS_GIT_OPEN_TIMEOUT:-5}"
            kill -TERM "$me" 2>/dev/null
        ) >/dev/null 2>&1 &
        w=$!
        ws_sha256 < "$1"
        rc=$?
        kill "$w" 2>/dev/null || true
        wait "$w" 2>/dev/null || true
        exit "$rc"
    ); then
        echo "error: reading '$1' did not complete within the bound \
(${WS_GIT_OPEN_TIMEOUT:-5}s) — refusing" >&2
        return 2
    fi
    printf '%s\n' "$out"
}
ws_tree_digest() {
    # $1 directory. Echoes one sha256 over (relative path, content) of
    # every regular file and (relative path, target) of every symlink,
    # in C-locale order — filenames NUL-delimited so no name can forge a
    # boundary. Digest bypass: a symlink's
    # TARGET is streamed byte-exact — `readlink -n` piped through
    # `od -tx1` into the digest, framed by an explicit NUL — never through
    # a command substitution, which strips trailing newlines and makes
    # `safe.cpp` and `safe.cpp\n` digest identically. Every file read
    # goes through the type-gated, bounded ws_file_sha256.
    # Each `while` is the last stage
    # of a pipeline, so bash runs it in its own subshell and the `exit 2`
    # inside only leaves THAT subshell. Without `|| exit 2` on the `done`,
    # control falls through to the next loop and the enclosing `( ... )`
    # returns the last loop's status — typically 0. So a failed
    # `ws_file_sha256` (an unreadable file, a type change mid-walk, the read
    # watchdog firing) is SWALLOWED: the digest is computed over a
    # TRUNCATED byte stream and reports success.
    #
    # Measured on this function without the guards: a tree with one unreadable
    # file digested to ce4f8839… with rc=0, and to 9b15201c… with rc=0 once
    # the file was readable — two different "successful" digests for one
    # tree. The false PASS follows: a file excluded at bind time stays
    # excluded at every recheck, so changes to it are never detected by the
    # content-binding gate this function exists to be.
    #
    # `set -euo pipefail` in run_matrix.sh does NOT rescue it — `-e` is
    # ignored inside a non-final pipeline component, and the outer status is
    # genuinely 0. The siblings already do this right (`ws_snapshot_inputs`
    # uses `done || exit 2`; `ws_results_digest` wraps its pipeline in an
    # explicit pipefail subshell).
    # The `|| exit 2` guards are the load-bearing half: without them the
    # `exit 2` never leaves the while-subshell at all, the inner `( ... )`
    # exits 0, and a caller's `pipefail` has nothing to catch. MEASURED
    # without them: an unreadable file in the tree gives rc=0 with a shorter digest.
    #
    # The explicit `set -o pipefail` covers the SECOND half directly. The
    # `( ... )` below is a NON-FINAL stage of `( ... ) | ws_sha256`, so its
    # status reaches the caller only under `pipefail`. run_matrix.sh and the
    # selftest both set `-o pipefail` at the top and subshells inherit it, so
    # inside the harness the guards alone would do — but this function is a
    # sourced library, and a caller that has NOT set it would otherwise get
    # `ws_sha256`'s 0 over a truncated stream (measured exactly that way).
    # Setting it here makes the refusal independent of the caller's options;
    # it is the same shape `ws_results_digest` already uses.
    (
        set -o pipefail
        (
            cd "$1" || exit 2
            find . -type f -print0 | LC_ALL=C sort -z | while IFS= read -r -d '' f; do
                printf 'F%s\0' "$f"
                ws_file_sha256 "$f" || exit 2
            done || exit 2
            find . -type l -print0 | LC_ALL=C sort -z | while IFS= read -r -d '' l; do
                printf 'L%s\0' "$l"
                # The target is streamed byte-exact (never through a
                # command substitution, which strips trailing newlines), so the
                # readlink failure is caught with a pipefail subshell rather than
                # by capturing it.
                ( set -o pipefail; readlink -n -- "$l" | od -An -v -tx1 ) || exit 2
                printf '\0'
            done || exit 2
        ) | ws_sha256
    )
}
require_ws_tree_digest() {
    # $1 directory, $2 expected digest, $3 stage label.
    local actual
    actual=$(ws_tree_digest "$1" || true)
    if [ "$actual" != "$2" ]; then
        echo "error: the contents of '$1' changed since they were bound \
(digest $2 -> ${actual:-unreadable}) during $3 — the analysis inputs were \
substituted; refusing the stage" >&2
        return 2
    fi
}
# The residual above, made concrete (reproduced):
# a same-user actor replaces an analyzed source (or compile_commands.json)
# AFTER analyze's identity check, the prover reads the replacement, and the
# original bytes are back before the post-stage digest — exit 0 over
# attacker-controlled inputs. The prover reads files by pathname and cannot
# be descriptor-bound, so the INPUTS move instead: the prover
# analyzes an IMMUTABLE VERIFIED COPY. ws_snapshot_inputs copies the
# approved tree + compile db into a fresh, private root this process just
# created (mktemp -d, mode 0700 — no other same-user path can pre-exist
# there), refusing symlinks and non-regular entries, and then re-digests
# the COPY and requires it to equal the APPROVED digests — that pin is what
# makes the copy the approved bytes and not whatever the live tree held
# during the copy. Every prover stage then runs against the snapshot only
# (ws_analyze_snapshot), the compile db copy's source paths rewritten to
# the snapshot, and the analysis outputs mapped back to the live paths for
# the consumers that act on the live tree. The live-tree digest checks
# after each stage remain as a cheap cross-check; they are not the
# guarantee. Known residual: the snapshot root itself is trusted because
# it is private and fresh — a same-user actor who can rename a directory
# this process created under a 0700 root it just minted is outside the
# threat model this harness has always used.
ws_copy_regular() {
    # $1 source file, $2 destination path. Type-gated + watchdog-bounded
    # read (the FIFO defense), create-new destination.
    if [ -L "$1" ] || [ ! -f "$1" ]; then
        echo "error: '$1' is not a regular file (it is a $(ws_object_kind "$1")) \
— refusing to snapshot it" >&2
        return 2
    fi
    (
        me=$BASHPID
        (
            sleep "${WS_GIT_OPEN_TIMEOUT:-5}"
            kill -TERM "$me" 2>/dev/null
        ) >/dev/null 2>&1 &
        w=$!
        set -o noclobber
        cat -- "$1" > "$2"
        rc=$?
        kill "$w" 2>/dev/null || true
        wait "$w" 2>/dev/null || true
        exit "$rc"
    ) || {
        echo "error: copying '$1' did not complete (a non-regular object or \
an occupied destination) — refusing" >&2
        return 2
    }
}
ws_snapshot_inputs() {
    # $1 live src dir, $2 approved src digest, $3 live compile_commands.json,
    # $4 approved compile-db sha256, $5 the fresh private snapshot root.
    # Populates $5/src (verified copy) and $5/ccdb/compile_commands.json
    # (verbatim copy — rewrite it afterwards with ws_rewrite_ccdb_paths).
    local root=$5
    if [ -L "$root" ] || [ ! -d "$root" ]; then
        echo "error: the snapshot root '$root' is not a real directory — refusing" >&2
        return 2
    fi
    mkdir -m 0700 "$root/src" "$root/ccdb" || return 2
    (
        cd "$1" || exit 2
        find . -type d -print0 | LC_ALL=C sort -z | while IFS= read -r -d '' d; do
            if [ -L "$d" ]; then
                echo "error: '$d' in the live tree is a symlink — refusing to snapshot" >&2
                exit 2
            fi
            # Sorted walk: every parent precedes its children, so a plain
            # mkdir suffices (`.` is the root itself, created above).
            if [ "$d" != "." ]; then
                mkdir -m 0700 "$root/src/$d" || exit 2
            fi
        done || exit 2
        find . -type f -print0 | LC_ALL=C sort -z | while IFS= read -r -d '' f; do
            ws_copy_regular "$f" "$root/src/$f" || exit 2
        done || exit 2
        if find . -type l -print -quit | grep -q .; then
            echo "error: the live tree holds a symlink — refusing to snapshot it" >&2
            exit 2
        fi
    ) || return 2
    # The COPY is the approved bytes — pinned against the digests approved
    # before any analysis, never against the live tree it was copied from.
    require_ws_tree_digest "$root/src" "$2" "the snapshot verification (sources)" || return 2
    ws_copy_regular "$3" "$root/ccdb/compile_commands.json" || return 2
    require_ws_file_digest "$root/ccdb/compile_commands.json" "$4" \
        "the snapshot verification (compile db)" || return 2
}
ws_rewrite_ccdb_paths() {
    # $1 compile_commands.json (the snapshot copy), $2 live src prefix,
    # $3 snapshot src prefix. Rewrites `file`, `command` and `arguments`;
    # `directory` (the build tree) stays as generated.
    python3 - "$1" "$2" "$3" <<'PYEOF_CCDB'
import json, sys
p, live, snap = sys.argv[1:4]
with open(p) as fh:
    db = json.load(fh)
def sub(v):
    return v.replace(live, snap) if isinstance(v, str) else v
for e in db:
    for k in ("file", "command"):
        if k in e:
            e[k] = sub(e[k])
    if "arguments" in e:
        e["arguments"] = [sub(a) for a in e["arguments"]]
with open(p, "w") as fh:
    json.dump(db, fh, indent=1)
PYEOF_CCDB
}
ws_map_snapshot_paths() {
    # $1 analysis JSON, $2 snapshot src prefix, $3 live src prefix. Maps
    # every string back to the live tree for the consumers that act on it
    # (assert_matrix.py, the stage-3b apply).
    python3 - "$1" "$2" "$3" <<'PYEOF_MAP'
import json, sys
p, snap, live = sys.argv[1:4]
with open(p) as fh:
    doc = json.load(fh)
def walk(v):
    if isinstance(v, str):
        return v.replace(snap, live)
    if isinstance(v, list):
        return [walk(x) for x in v]
    if isinstance(v, dict):
        return {k: walk(x) for k, x in v.items()}
    return v
with open(p, "w") as fh:
    json.dump(walk(doc), fh, indent=2)
PYEOF_MAP
}
ws_analyze_snapshot() {
    # $1 snapshot root, $2 prover binary, $3 TU path relative to src.
    # The prover sees ONLY the snapshot: its compile db, its src root, its
    # copy of the TU.
    "$2" -p "$1/ccdb" --src-root="$1/src" "$1/src/$3"
}
ws_snapshot_seal() {
    # $1 snapshot root. The prover opens the snapshot's files by
    # PATHNAME, so a same-UID writer could replace a TU inside the
    # snapshot while the prover runs, and the LIVE post-stage digests
    # would never notice. Seal it: every file AND directory loses its
    # write bits (an entry cannot be created, renamed or replaced inside a
    # directory the writer cannot write), and the root goes 0500. This is
    # PREVENTION for an unprivileged writer only — a same-UID attacker can
    # chmod it back — so every stage also re-checks the seal before the
    # prover runs and re-digests the snapshot ITSELF afterwards
    # (require_ws_snapshot_intact): against a same-UID writer the defense
    # is DETECTION plus the fresh private root, not prevention, unless the
    # prover consumes descriptors.
    chmod -R a-w "$1" || return 2
    chmod 0500 "$1" || return 2
    require_ws_snapshot_sealed "$1" "the seal"
}
require_ws_snapshot_sealed() {
    # $1 snapshot root, $2 stage label. Any writable entry (owner, group
    # or other write bit) anywhere under the root — the root included —
    # means the seal was broken.
    local hit
    hit=$(find "$1" \( -perm -0200 -o -perm -0020 -o -perm -0002 \) -print -quit 2>/dev/null)
    if [ -n "$hit" ]; then
        echo "error: '$1' is not sealed — '$hit' is writable before $2 — refusing" >&2
        return 2
    fi
}
require_ws_snapshot_intact() {
    # $1 snapshot root, $2 approved src digest, $3 digest of the REWRITTEN
    # compile db copy, $4 stage label. The snapshot's own post-stage
    # content bind: seal intact, sources and compile db byte-identical to
    # what was approved — an in-snapshot substitution fails the stage even
    # when its live twin was left untouched.
    require_ws_snapshot_sealed "$1" "$4" || return 2
    require_ws_tree_digest "$1/src" "$2" "$4 (snapshot sources)" || return 2
    require_ws_file_digest "$1/ccdb/compile_commands.json" "$3" "$4 (snapshot compile db)" || return 2
}
ws_snapshot_unseal() {
    # $1 snapshot root. Owner write bits back — for deletion only.
    chmod -R u+w "$1" 2>/dev/null || true
}
ws_capture_result() {
    # $1 destination (a CREATE-NEW file inside a results dir this run
    # created), then the producer command.
    # The prover's output is as much an analysis input as its
    # sources — assert_matrix.py and the apply step read it back — so it
    # is written ONLY into a fresh private results dir (never a
    # pre-existing or operator-visible path), as a create-new file (a
    # pre-placed file at the destination refuses), to be sealed and
    # digested the moment the stage's producer runs end.
    #
    # `noclobber` does NOT make this open bounded. MEASURED:
    # `set -o noclobber; echo x > fifo` on an existing writerless FIFO
    # BLOCKS — the open waits for a reader before noclobber's check can
    # speak — so a FIFO planted at a result pathname would wedge the matrix
    # forever, before the prover runs. Every READ seam
    # and every watchdog is bounded, and `ws_copy_regular` bounds its write;
    # this write seam is bounded the same way. It carries
    # the same two gates as `ws_copy_regular`: an ABSENCE check (a
    # create-new destination is one that does not exist — that refuses a
    # FIFO, a symlink and a pre-placed file with one accurate message,
    # strictly more than `noclobber` caught) and the watchdog, so an object
    # planted between the check and the redirect is a bounded, loud failure
    # rather than a hang. `noclobber` stays as the belt that narrows that
    # same race.
    local out=$1
    shift
    if [ -e "$out" ] || [ -L "$out" ]; then
        echo "error: the result destination '$out' already exists (it is a \
$(ws_object_kind "$out")) — results are written create-new into a fresh \
private dir; refusing" >&2
        return 2
    fi
    (
        me=$BASHPID
        (
            sleep "${WS_GIT_OPEN_TIMEOUT:-5}"
            kill -TERM "$me" 2>/dev/null
        ) >/dev/null 2>&1 &
        w=$!
        # The watchdog bounds the OPEN ONLY — never the producer. The
        # producer here is the PROVER (clang LibTooling over a translation
        # unit), which legitimately runs for many seconds; a bound around it
        # would kill every real analysis. So the destination is opened onto
        # its own descriptor under the watchdog, the watchdog is reaped, and
        # only then does the producer run, writing to that descriptor with no
        # time limit. Same shape as ws_open_gitdir_in_cwd's bounded gitdir
        # open, and the brace group scopes the stderr redirect to the open
        # attempt while the descriptor persists.
        ofd=''
        set -o noclobber
        { exec {ofd}>"$out"; } 2>/dev/null || true
        kill "$w" 2>/dev/null || true
        wait "$w" 2>/dev/null || true
        if [ -z "$ofd" ]; then
            exit 2
        fi
        "$@" >&"$ofd"
        rc=$?
        exec {ofd}>&-
        exit "$rc"
    ) || {
        echo "error: capturing the result at '$out' failed — the destination \
could not be opened within the bound (${WS_GIT_OPEN_TIMEOUT:-5}s) or the \
producer failed; refusing" >&2
        return 2
    }
}
ws_results_seal() {
    # $1 results dir. The snapshot's seal: files AND the directory lose
    # their write bits, the directory goes 0500 — an unprivileged writer
    # can no longer create, rename or replace an entry under it.
    chmod -R a-w "$1" || return 2
    chmod 0500 "$1" || return 2
    require_ws_snapshot_sealed "$1" "the results seal"
}
ws_results_digest() {
    # $1 results dir, $2 the EXPECTED file set (one basename per line).
    # path + sha256 over EXACTLY that set: an extra entry of any kind, or
    # a missing one, refuses — a consumer must never glob up a forged
    # extra or silently miss a result.
    local dir=$1 expected=$2 actual want
    actual=$(cd "$dir" && find . -mindepth 1 -print | LC_ALL=C sort) || return 2
    want=$(printf '%s\n' "$expected" | sed -e '/^$/d' -e 's#^#./#' | LC_ALL=C sort)
    if [ "$actual" != "$want" ]; then
        echo "error: the result set in '$dir' is not the expected set — \
expected [$(printf '%s' "$want" | tr '\n' ' ')] found [$(printf '%s' "$actual" | tr '\n' ' ')] \
— an extra or missing result; refusing" >&2
        return 2
    fi
    (
        set -o pipefail
        (
            cd "$dir" || exit 2
            printf '%s\n' "$expected" | sed '/^$/d' | LC_ALL=C sort | while IFS= read -r n; do
                printf 'F%s\0' "$n"
                ws_file_sha256 "$n" || exit 2
            done
        ) | ws_sha256
    )
}
require_ws_results_intact() {
    # $1 results dir, $2 the digest taken when it was sealed, $3 the
    # expected file set, $4 stage label. Run IMMEDIATELY before a
    # consumer reads and again after it returns: seal intact, set exact,
    # every byte as sealed. A same-UID writer can chmod the seal away, so
    # against that writer this is DETECTION (plus the fresh private
    # root), not prevention.
    require_ws_snapshot_sealed "$1" "$4" || return 2
    local actual
    if ! actual=$(ws_results_digest "$1" "$3"); then
        echo "error: the results in '$1' could not be re-verified during $4 — refusing the stage" >&2
        return 2
    fi
    if [ "$actual" != "$2" ]; then
        echo "error: the results in '$1' changed since they were sealed \
(digest $2 -> $actual) during $4 — a result was forged or replaced; refusing the stage" >&2
        return 2
    fi
}

require_ws_file_digest() {
    # $1 file, $2 expected sha256, $3 stage label. Through the
    # type-gated, bounded reader — a FIFO refuses, never blocks.
    local actual
    actual=$(ws_file_sha256 "$1" || true)
    if [ "$actual" != "$2" ]; then
        echo "error: '$1' changed since it was bound (sha256 $2 -> \
${actual:-unreadable}) during $3 — the analysis inputs were substituted; \
refusing the stage" >&2
        return 2
    fi
}

# Re-verify the OBJECT at a pathname still carries the
# identity bound at setup, before a stage that EXECUTES workspace
# content. ws_git does not need this by pathname — it
# checks `.` after binding its cwd and the gitdir through its open
# descriptor — but the NON-git stages (colcon, the cargo apply, the
# built matrix_runner) take pathnames a shell cannot bind, so for them a
# pathname swap after the bind is stopped here and the remaining residual
# is the check→exec instant itself — the same tier language as
# run_matrix.sh's own residuals.
require_ws_identity() {
    # $1 path to check, $2 expected dev:inode, $3 stage label.
    local actual
    actual=$(ws_identity "$1" 2>/dev/null || true)
    if [ "$actual" != "$2" ]; then
        echo "error: '$1' no longer carries the bound identity \
($2 -> ${actual:-unreadable}) before $3 — another process replaced it; \
refusing to execute inside it" >&2
        return 2
    fi
}

# The combined check for every post-init non-git stage — the
# workspace AND its gitdir must both still carry the identities this run
# bound. The gitdir half is what makes the filter reasoning above hold:
# with system/global config nulled, a filter command can only come from
# inside .git, and a swapped .git no longer carries the bound identity.
# In-place edits INSIDE the bound gitdir remain the known residual
# (a same-UID writer editing another process's mktemp scratch is outside
# the threat model — that actor already executes as the user).
require_ws_and_gitdir() {
    # $1 workspace path, $2 ws dev:inode, $3 gitdir dev:inode, $4 label.
    # Both are PATHNAME checks — lstat-gated (a symlink to the
    # moved original carries the identity but is refused).
    require_ws_path_identity "$1" "$2" "$4" || return 2
    require_ws_path_identity "$1/.git" "$3" "$4 (gitdir)" || return 2
}

# ---- prover mutant binaries ---------------------------
#
# `run_matrix.sh` stage 5 builds each mutant prover as its OWN binary beside
# the production one and deletes it once the mutant has been judged. That
# deletion sat on the paths THROUGH the judgement — so an abort MID-mutant
# (a `set -e` failure, an `exit 2` from a bound-input re-check, a Ctrl-C)
# left `cerulion-ros2-migrate-clang-mutant-<name>` sitting in the tool
# directory. The tool directory is the repo checkout, bind-mounted into the
# container, so that leftover outlives the container: a prover with a proof
# compiled OUT, sitting where an operator (or a later run) reaches for "the
# tool".
#
# The guarantee these give run_matrix.sh, on EVERY exit path: no mutant
# binary this run built survives it, and the production prover a later run
# will use is byte-for-byte the one this run built.
#
# They live here rather than in run_matrix.sh so run_matrix_selftest.sh can
# drive them DESK-SIDE, without a container — the same reason every other
# helper in this file does.
#
# Neutralized at load, so this run's bindings are the only ones the restore
# ever sees. Deliberately a plain assignment rather than ws_bind_publish's
# `unset` + loud disown: every read in this module is unguarded (`set -u` is
# on in run_matrix.sh), so `unset` would need a `:-` at four sites to buy a
# warning about a variable no supported caller sets. Note the difference
# from that disown, which this does NOT close: an inherited value that arrived EXPORTED
# stays exported, so these names are visible to children (build.sh, the
# prover, python3). Harmless — nothing reads them — but the claim here is
# neutralization, not a disown.
WS_MUTANT_PROD=""
WS_MUTANT_PRISTINE=""
WS_MUTANT_BUILT=""
WS_MUTANT_DIR=""
WS_MUTANT_MODE=""
# Artifact identities: "<dev:inode:size:mtime_ns> <path>" per BUILT mutant (see
# ws_mutant_bind_identity); the pathname list above is the registry, this is
# what makes a registered NAME removable only while it still carries the
# artifact this run built.
WS_MUTANT_IDENTS=""
# The build-bind gap:
# this run's PRIVATE mutant root (a mktemp -d under the tool
# directory that no other run can name — ws_mutant_root_create) and the
# run-owned mtime STAMP every bound identity must carry (see the identity
# block below — minted by ws_mutants_bind, applied by ws_stamp_artifact).
WS_MUTANT_ROOT=""
WS_MUTANT_STAMP_NS=""
WS_MUTANT_STAMP_MTIME=""
# The exclusive per-tool-directory run lock (ws_run_lock): the descriptor
# this shell holds it on — the tool DIRECTORY's own — and the
# directory's path.
WS_RUN_LOCK_FD=""
WS_RUN_LOCK_PATH=""
# STICKY: a restore that could not put the tool directory back must fail the
# run. It cannot do that by returning non-zero (that aborts the rest of the
# EXIT trap), so it latches here and run_matrix.sh promotes it to the exit
# status AFTER every other cleanup has run.
WS_MUTANT_RESTORE_FAILED=""

# The octal permission bits of a path (GNU stat, then BSD).
ws_object_mode() {
    # GNU first, then BSD — ws_identity's idiom exactly, which silences the
    # FIRST probe's usage message and leaves the second's error visible.
    # Silencing both would turn "neither stat worked" into
    # silence and let the caller fabricate a mode with no diagnostic.
    stat -c '%a' "$1" 2>/dev/null || stat -f '%Lp' "$1"
}

ws_mutant_stale_list() {
    # $1 tool directory. Echoes one path per line for every mutant prover
    # binary already sitting there (empty when clean).
    local f
    for f in "$1"/cerulion-ros2-migrate-clang-mutant-*; do
        # `-e` follows symlinks: a DANGLING one would be invisible here, and
        # this list feeds BOTH the startup refusal and the end-of-run sweep —
        # the module's only backstop for a SIGKILL or a hand-built mutant. The
        # registered-removal loop got this guard at the same time; its
        # sibling needs it for the same reason, one scope out.
        [ -e "$f" ] || [ -L "$f" ] || continue
        printf '%s\n' "$f"
    done
}

ws_mutant_stale_roots() {
    # $1 tool directory. Echoes one path per line for every PRIVATE mutant
    # root (`.mutants.*`, ws_mutant_root_create's shape) sitting there that is
    # not this run's own. Under the run lock any such root belongs to a run
    # that did not clean up — a SIGKILL, a power cut, a `docker kill` — and it
    # is refused at startup BY NAME, never entered and never deleted: what it
    # holds is a prover with a proof compiled OUT, and this run did not
    # create it. `-e` follows symlinks, so a dangling link is listed through
    # `-L` too, exactly as ws_mutant_stale_list lists one.
    local d
    for d in "$1"/.mutants.*; do
        [ -e "$d" ] || [ -L "$d" ] || continue
        [ "$d" != "$WS_MUTANT_ROOT" ] || continue
        printf '%s\n' "$d"
    done
}

ws_mutants_require_clean() {
    # $1 tool directory. A mutant binary already on disk is REFUSED, not
    # deleted: this harness never removes a path it did not create, and a
    # leftover is precisely the artifact that makes the next run's verdict
    # untrustworthy — so it is named and the operator decides. A
    # private mutant root left by an earlier run is refused the same way.
    local stale line
    stale=$(ws_mutant_stale_list "$1")
    if [ -n "$stale" ]; then
        echo "error: mutant prover binaries are already present in '$1':" >&2
        while IFS= read -r line; do
            [ -n "$line" ] || continue
            echo "  $line" >&2
        done <<< "$stale"
        echo "a mutant is a prover with a proof compiled OUT and must never \
outlive the matrix that built it. This run did not create these, so it will \
not delete them — remove them by hand and re-run." >&2
        return 2
    fi
    stale=$(ws_mutant_stale_roots "$1")
    if [ -n "$stale" ]; then
        echo "error: private mutant root(s) left by an earlier run are present in '$1':" >&2
        while IFS= read -r line; do
            [ -n "$line" ] || continue
            echo "  $line" >&2
        done <<< "$stale"
        echo "each is where a run that did not clean up built its mutants — provers \
with a proof compiled OUT. No live run owns one while the run lock is held \
(run_matrix.sh takes it before this check), and this run did not create them, \
so it will not delete them — remove them by hand and re-run." >&2
        return 2
    fi
    return 0
}

ws_run_lock_precheck() {
    # $1 tool directory. The cheap BY-NAME gate: a link or a non-directory
    # at the path is refused instantly and named.
    # This is NOT what the lock's identity rests on — a by-name check
    # is check-then-use, and a swap landing after it would be followed by
    # the open — so ws_run_lock verifies the object AFTER the open, through
    # the descriptor. This exists for the diagnosis, and selftest 31(p)
    # overrides it to model a swap landing in exactly that gap.
    if [ -L "$1" ] || [ ! -d "$1" ]; then
        echo "error: ws_run_lock: '$1' is not a directory (it is a \
$(ws_object_kind "$1")) — refusing to lock it" >&2
        return 2
    fi
}

ws_run_lock() {
    # $1 tool directory. The build-bind gap:
    # ONE matrix run per tool directory at a time, refused loudly at startup.
    # The mutants are private to a run (ws_mutant_root_create), but the
    # production prover and its restore share one pathname in this
    # directory, and two runs building it under each other cannot both be
    # right.
    #
    # The lock is taken on the DIRECTORY
    # ITSELF, never on a lock FILE. A lock file is a NAME: checking that a
    # `.run_matrix.lock` is a regular file and then opening it by
    # redirection lets a symlink swapped in between the two be followed, and
    # two runs can flock two different objects — the build-bind race this
    # lock exists to end, reopened. A directory opened on a descriptor is
    # the object, not a name: after the open the descriptor is verified
    # (fstat: a directory; lstat of the name: not a link, the same
    # dev:inode) and `flock(2)` is taken on THAT descriptor, so a swap
    # landing before the open is refused and one landing after it cannot
    # move the lock. Nothing is created on disk. `flock` through python's
    # fcntl on a descriptor THIS SHELL keeps open: the lock lives on the open
    # file description, holds for as long as the shell (and any child that
    # inherited the descriptor) lives, and the kernel releases it on exit,
    # however the run ends — no stale lock, nothing to clean. Non-blocking:
    # a second run is told what holds the directory and stops. The
    # inherited-descriptor residual, stated: a child that outlives the run
    # keeps the lock until it exits; every child this script starts is
    # bounded (the read watchdogs' `sleep` by their own timeout).
    local fd rc=0
    if [ -n "$WS_RUN_LOCK_FD" ]; then
        echo "error: ws_run_lock: this shell already holds the lock on \
'$WS_RUN_LOCK_PATH'" >&2
        return 2
    fi
    ws_run_lock_precheck "$1" || return 2
    if ! exec {fd}<"$1"; then
        echo "error: ws_run_lock: could not open '$1' to lock it" >&2
        return 2
    fi
    python3 - "$fd" "$1" <<'PYEOF_LOCK' || rc=$?
import fcntl, os, stat, sys
fd = int(sys.argv[1])
path = sys.argv[2]
opened = os.fstat(fd)
if not stat.S_ISDIR(opened.st_mode):
    sys.exit(70)
named = os.lstat(path)
if stat.S_ISLNK(named.st_mode) or (opened.st_dev, opened.st_ino) != (named.st_dev, named.st_ino):
    sys.exit(71)
try:
    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    sys.exit(75)
PYEOF_LOCK
    case $rc in
        0) ;;
        70|71)
            exec {fd}>&-
            echo "error: ws_run_lock: the object opened at '$1' is not the directory \
that name denotes ($( [ "$rc" = 70 ] && echo "the descriptor is not a directory" \
|| echo "the name is a link, or was swapped between the check and the open")) — \
refusing to lock through it" >&2
            return 2
            ;;
        75)
            exec {fd}>&-
            echo "error: another run_matrix.sh is live in '$1' (it holds the lock on \
that directory) — one matrix run per tool directory at a time: the production \
prover and its restore share one pathname there. Wait for that run to finish; \
the lock releases itself when it exits, however it exits." >&2
            return 2
            ;;
        *)
            exec {fd}>&-
            echo "error: ws_run_lock: could not take the lock on '$1' (python3 \
fcntl.flock failed, rc=$rc) — refusing to run without the lock" >&2
            return 2
            ;;
    esac
    WS_RUN_LOCK_FD=$fd
    WS_RUN_LOCK_PATH=$1
}

# The pristine copy of
# the production prover is taken THROUGH AN OPEN DESCRIPTOR, never by
# re-resolving the pathname. Gating the pathname's
# type and then `cp`ing it does not hold the gate across the copy: a symlink
# swapped in at that instant makes the "pristine" copy an arbitrary target
# that the restore would later write back as "the prover".
#
# The open AND the read run inside a SUBSHELL whose OWN pid the watchdog
# targets. That is `ws_file_sha256`'s shape, deliberately NOT
# `ws_open_gitdir_in_cwd`'s: a TERM then kills only the subshell and this
# function REFUSES with a diagnostic and a status, instead of killing its
# caller. The difference is load-bearing, and MEASURED: bash does NOT run an
# EXIT trap when it is TERMed while blocked in a BUILTIN (`exec {fd}< ` on a
# writerless FIFO gives rc 143 and no trap), while the same TERM delivered
# while it waits on a CHILD does run it — which is the shape run_matrix.sh's
# own measurement at the top of this module rests on. A caller-pid watchdog
# here would therefore skip run_matrix.sh's entire EXIT cleanup, silently.
#
# The OPENED object is then stat'd through `/dev/fd/N` (devfs on macOS, a
# procfs magic link on Linux; both report the open object's type), and the
# bytes are read from that descriptor, so a pathname swap after the open
# cannot change what is read — the descriptor is bound to the inode.
# MEASURED on macOS for the type test and the inode binding; the Linux
# procfs half is by construction and the same mechanism this module already
# relies on at the gitdir open, not something measured here.
#
# The pathname is gated for SYMLINKS because symlink-ness is the one
# property no descriptor can report: open(2) follows the link, so the fd
# would faithfully describe the TARGET. The caller ALSO gates the pathname's
# type before calling (see `ws_mutants_bind`) so a FIFO sitting at the
# prover path is refused instantly rather than after the bound; the
# descriptor test is not redundant behind that, because it is what holds
# across the open for a swap landing in the window, and the selftest drives
# this function DIRECTLY so the caller's gate cannot mask it.
#
# Residual, stated at its real strength: a swap landing between the symlink
# gate and the open that puts another regular file — or a symlink to one,
# since the symlink gate is itself a pathname test and as raceable as any —
# at the name is copied. That is not a TOCTOU on this copy; it is
# indistinguishable from the prover having been replaced a moment earlier,
# which this boundary does not claim to detect (`ws_mutants_require_clean`
# and the build own that).
#
# Identity is deliberately NOT compared against the descriptor: measured on
# macOS, `stat -L /dev/fd/N` reports the DEVFS device rather than the
# file's, so a dev:inode compare would false-refuse on every desk run. The
# gitdir half never meets this because `ws_git_fd_binding_available` — a
# TRAVERSABILITY probe, for a different reason — already sends macOS down
# its by-name identity path.
#
# Test seam (verification only): parks between the OPEN and the READ so the
# selftest can swap the pathname deterministically and prove the bytes come
# from the descriptor. Its own variable, not `ws_pause_seam`'s, so an arm
# parking a bound wrapper cannot also park here. Inert unless set.
ws_copy_pause_seam() {
    if [ -n "${CERULION_WS_COPY_PAUSE_DIR:-}" ]; then
        # Say so: an INHERITED value parks a production bind, and the watchdog
        # then refuses with "did not complete within the bound", which names
        # the wrong cause entirely.
        echo "note: CERULION_WS_COPY_PAUSE_DIR is set; parking at the \
verification seam" >&2
        echo paused > "$CERULION_WS_COPY_PAUSE_DIR/ready"
        read -r _ < "$CERULION_WS_COPY_PAUSE_DIR/go" || true
    fi
}

ws_copy_through_descriptor() {
    # $1 source, $2 destination, $3 label. Copies $1 to $2 only if the
    # OPENED object is a regular file. The open AND the read are bounded by
    # WS_GIT_OPEN_TIMEOUT.
    #
    # $2 must be a caller-minted PRIVATE path. The destination is written
    # THROUGH, not gated: a symlink there is FOLLOWED, as the `cp` this
    # replaced also did. The equivalence stops there, and the one place it
    # breaks is guarded below rather than documented away — `cp a a` REFUSES
    # ("are identical"), whereas `> "$2"` would truncate the very inode the
    # descriptor is open on, so `cat` would read zero bytes and this function
    # would report SUCCESS over an empty copy having destroyed its own
    # source. That is the worst outcome this module has (the restore would
    # `mv` a 0-byte file over the production prover and chmod it executable),
    # so same-object is refused explicitly, and a copy that somehow still
    # lands empty is refused as a backstop.
    if [ -L "$1" ]; then
        echo "error: '$1' is a SYMLINK before $3 — refusing to read through \
it (a descriptor cannot report symlink-ness: the open would follow it)" >&2
        return 2
    fi
    if [ -e "$2" ] && [ "$1" -ef "$2" ]; then
        echo "error: the destination '$2' is the SAME OBJECT as the source \
'$1' before $3 — refusing: the write would truncate the source through the \
descriptor already open on it and then report success over an empty copy" >&2
        return 2
    fi
    local st=0
    # `|| st=$?`, never a bare `( … )`: a bare compound command returning
    # non-zero is an errexit TRIGGER, so under `set -e` the shell would die AT
    # the subshell and neither this assignment nor the `case` below would run —
    # the refusal this function exists to deliver would never fire, and the raw
    # internal status would escape. MEASURED on the shipped `set -euo pipefail`.
    # `ws_file_sha256` avoids it by being an `if !` condition; this one says so
    # explicitly instead of depending on its callers.
    (
        # The watchdog TERMs this subshell. Catching it gives the bound its OWN
        # status, so the `*)` arm below can be an accurate "unexpected" rather
        # than a timeout claim about a SIGSEGV or a missing `cat` — and it
        # stops bash printing a reconstruction of this whole subshell body to
        # the caller's stderr ahead of the refusal, which read like an internal
        # crash.
        # BOTH ends are opened HERE, by this subshell, under the watchdog —
        # `cat` never performs a redirection of its own. That is what makes
        # the bound reach a destination that blocks: an open is a builtin in
        # this shell and a TERM interrupts it, whereas a child blocked in its
        # OWN redirection cannot be interrupted by a trap at all
        # (measured: with `cat > "$2"` the subshell waits for it forever
        # and the bound never fires). Backgrounding `cat` instead is the
        # WRONG fix and was measured to be worse: while blocked in its own
        # open its fd 1 is still the caller's inherited pipe, so it pins a
        # command substitution open exactly as a watchdog without the stdout redirect does.
        # With no child of its own to orphan, nothing can outlive the refusal.
        #
        # Residual, narrower and stated: a `cat` already streaming into a
        # destination whose reader then STOPS is a foreground block the trap
        # cannot interrupt. Both opens are bounded; a mid-stream stall is not.
        trap 'exit 6' TERM
        me=$BASHPID
        (
            sleep "${WS_GIT_OPEN_TIMEOUT:-5}"
            kill -TERM "$me" 2>/dev/null
        ) >/dev/null 2>&1 &
        w=$!
        fd=''
        ofd=''
        rc=0
        # The brace group scopes the stderr redirect to the open attempt, as
        # at the gitdir open; a failed open leaves $fd empty.
        { exec {fd}< "$1"; } 2>/dev/null || true
        if [ -z "$fd" ]; then
            rc=3
        elif [ ! -f "/dev/fd/$fd" ]; then
            rc=4
        else
            ws_copy_pause_seam
            ofd=
            { exec {ofd}> "$2"; } 2>/dev/null || true
            if [ -z "$ofd" ]; then
                rc=5
            else
                cat <&"$fd" >&"$ofd" || rc=5
                exec {ofd}>&-
            fi
        fi
        kill "$w" 2>/dev/null || true
        wait "$w" 2>/dev/null || true
        exit "$rc"
    ) || st=$?
    case $st in
        0)
            # Backstop for the short/empty-copy shape: nothing here checks a
            # size or a digest, and an empty "pristine" prover is the one
            # outcome the restore turns into a corrupt-but-executable binary.
            if [ -s "$1" ] && [ ! -s "$2" ]; then
                echo "error: the copy of '$1' into '$2' is EMPTY before $3, \
though the source is not — refusing rather than banking an unusable \
snapshot" >&2
                return 2
            fi
            return 0
            ;;
        3) echo "error: '$1' could not be OPENED before $3 (the pathname is a \
$(ws_object_kind "$1")) — if that says regular file, the open itself failed, \
which is a permission or path problem rather than a type one; refusing" >&2 ;;
        4) echo "error: '$1' opened, but the DESCRIPTOR is not a regular file \
(the pathname is a $(ws_object_kind "$1")) before $3 — if that says regular \
file, /dev/fd may be unavailable on this host; refusing" >&2 ;;
        5) echo "error: copying '$1' (read through its opened descriptor) into \
'$2' failed before $3 — the source read or the destination write did not \
complete (a full filesystem, an unwritable or substituted destination); \
refusing" >&2 ;;
        6) echo "error: opening and reading '$1' did not complete within the \
bound (${WS_GIT_OPEN_TIMEOUT:-5}s) before $3 — refusing" >&2 ;;
        *) echo "error: copying '$1' before $3 ended with an unexpected \
internal status ($st) — refusing" >&2 ;;
    esac
    return 2
}

ws_mutants_bind() {
    # $1 tool directory, $2 the production prover binary. Takes a private
    # copy of the production binary OUTSIDE the tool directory so the
    # restore can prove the binary a later run will use is this one.
    if [ -n "$WS_MUTANT_PRISTINE" ]; then
        # A second bind would leak the first pristine copy AND clear
        # WS_MUTANT_BUILT, orphaning every mutant registered against the
        # first bind — those paths would then be removed by nothing.
        echo "error: ws_mutants_bind: a production prover is already bound \
($WS_MUTANT_PROD) — restore before binding again" >&2
        return 2
    fi
    ws_mutants_require_clean "$1" || return 2
    # The cheap pathname TYPE gate. A FIFO simply SITTING
    # at the prover path is a steady state, not a race, and this refuses it
    # INSTANTLY and BY NAME instead of stalling the bind for a whole
    # WS_GIT_OPEN_TIMEOUT and then reporting the descriptor branch's hedged
    # wording — the same trade `ws_open_gitdir_in_cwd` makes for `.git`.
    # (The gate is NOT load-bearing for
    # run_matrix.sh's EXIT-trap cleanup. It would be if the
    # watchdog TERMed the CALLER; it TERMs a subshell instead (below),
    # and MEASURED, a FIFO here refuses cleanly with the
    # EXIT trap intact even with this gate deleted. The gate earns its place
    # on promptness and diagnosis, not on cleanup.)
    # It does NOT make the descriptor test redundant: that one holds across
    # the open for a swap landing in the window, and the selftest drives
    # ws_copy_through_descriptor DIRECTLY so this gate cannot mask it.
    if [ -L "$2" ] || [ ! -f "$2" ]; then
        echo "error: '$2' is not a regular file (it is a $(ws_object_kind "$2")) \
— refusing to bind the production prover" >&2
        return 2
    fi
    local copy
    copy=$(mktemp "${TMPDIR:-/tmp}/cerulion_migrate_prover.XXXXXX") || return 2
    # The prover's REAL mode, captured before the copy. The snapshot reads
    # through a descriptor and writes BYTES, so no mode rides THAT copy. The
    # restore still uses `cp`, into a fresh `mktemp` — and `cp` leaves an
    # existing destination's mode alone (mktemp creates at 0600), so the
    # bound mode must be re-applied explicitly there; restoring correct
    # bytes as a non-executable file is the failure to avoid.
    if ! WS_MUTANT_MODE=$(ws_object_mode "$2"); then
        echo "warning: could not read '$2' mode; a restore will use 0755" >&2
        WS_MUTANT_MODE=0755
    fi
    if ! ws_copy_through_descriptor "$2" "$copy" \
        "binding the production prover"; then
        rm -f "$copy"
        echo "error: could not copy the production prover '$2' aside" >&2
        return 2
    fi
    chmod 0400 "$copy" ||
        echo "warning: the pristine prover copy could not be made read-only \
($copy); it is still a faithful copy, only less protected" >&2
    WS_MUTANT_PROD=$2
    WS_MUTANT_PRISTINE=$copy
    WS_MUTANT_DIR=$1
    WS_MUTANT_BUILT=""
    WS_MUTANT_IDENTS=""
    # The run's stamp — 2001-09-09 plus this shell's pid, in whole
    # seconds, as nanoseconds and as the `%.9Y`/`%Fm` rendering the identity
    # carries. Unique among live runs (pids are), and a value no clock
    # produces for a write that happens now (see the identity block).
    WS_MUTANT_STAMP_NS=$(( (1000000000 + $$) * 1000000000 ))
    WS_MUTANT_STAMP_MTIME="$((1000000000 + $$)).000000000"
}

ws_mutant_root_create() {
    # The build-bind gap: mint this run's
    # PRIVATE mutant root under the bound tool directory. Every mutant is
    # built INTO it (build.sh --out-dir) and identity-bound there, so the
    # object bound was never at a name another run could write — the gap
    # between a build at the shared name and the identity read, which a
    # concurrent run could land in (binding the OTHER run's artifact as this
    # run's, then deleting it at cleanup), does not exist for a name only
    # this run knows. `mktemp -d`: random, created atomically, mode 0700.
    # Requires the bind (the restore is what removes the root — without a
    # bind nothing would) and refuses a second root (the first would be
    # orphaned with whatever it holds).
    if [ -z "$WS_MUTANT_DIR" ] || [ -z "$WS_MUTANT_PRISTINE" ]; then
        echo "error: ws_mutant_root_create: no production prover bound \
(call ws_mutants_bind first)" >&2
        return 2
    fi
    if [ -n "$WS_MUTANT_ROOT" ]; then
        echo "error: ws_mutant_root_create: a private mutant root is already \
minted ($WS_MUTANT_ROOT) — restore before minting another" >&2
        return 2
    fi
    local root
    if ! root=$(mktemp -d "$WS_MUTANT_DIR/.mutants.XXXXXX"); then
        echo "error: ws_mutant_root_create: could not create a private mutant \
root under '$WS_MUTANT_DIR'" >&2
        return 2
    fi
    WS_MUTANT_ROOT=$root
}

ws_mutant_register() {
    # $1 the mutant binary path about to be built. Registered BEFORE the
    # build, so a binary an interrupted build left half-written is NAMED by
    # the sweep (never removed blind — an entry with no identity
    # is reported and latched, see ws_mutant_remove). Only registered paths
    # are ever removed.
    #
    # Refuses without a bind, in ws_bind_gitdir's spelling, because the
    # half-working alternative is the worse one: the removal half reads
    # WS_MUTANT_BUILT alone and would keep working, while the pristine half
    # would silently not exist — a caller would get one of this module's two
    # guarantees and no sign the other was missing.
    if [ -z "$WS_MUTANT_PROD" ] || [ -z "$WS_MUTANT_PRISTINE" ]; then
        echo "error: ws_mutant_register: no production prover bound \
(call ws_mutants_bind first)" >&2
        return 2
    fi
    # Registered ⇒ UNBOUND until its build completes: an
    # identity still on record from a previous life of this name — a name
    # re-registered with no removal in between — would otherwise make
    # ws_mutant_remove report the half-written product of an interrupted
    # rebuild as a SWAP ("something replaced it behind the pathname") and
    # send the operator looking for an intruder, when what happened is a
    # build that never completed (refused either way, but for the
    # true reason). Cleared here, at the one place a name's life begins.
    WS_MUTANT_IDENTS=$(ws_mutant_idents_without "$1")
    # ONE registration per name: a name re-registered with no
    # removal in between would otherwise appear twice in the registry, and a sweep
    # that refuses rather than removes would then refuse it twice — the same
    # object reported twice as two leftovers. The earlier line is dropped
    # before this one is appended; the registry is a set of names.
    local l kept=""
    while IFS= read -r l; do
        [ -n "$l" ] || continue
        [ "$l" = "$1" ] && continue
        kept="$kept$l
"
    done <<< "$WS_MUTANT_BUILT"
    WS_MUTANT_BUILT="${kept}${1}
"
}

# ---- artifact identity ---------------------------------------------------
# A registry keyed on a PATHNAME is not enough: `ws_mutant_register` records
# the path before the build, and a removal by name — the per-mutant cleanup in
# run_matrix.sh and the EXIT-trap sweep in `ws_mutants_restore` — deletes
# whatever the name denotes at that instant. With two runs over one tool
# directory, run 1's cleanup deletes run 2's live mutant and corrupts its
# verdict (reproduced). The registry therefore binds the ARTIFACT:
# right after a mutant is built, `ws_mutant_bind_identity` records the
# dev:inode:size:mtime(ns) of the object at the name, and every removal
# refuses a name that no longer carries it — reported, never deleted, because
# an object that is not the one this run built is not this run's to delete.
#
# Identity, NOT a content digest, and the difference is the whole point: a
# peer run's rebuild of the same mutant is byte-identical (same source, same
# flags), so a digest would ACCEPT exactly the collision this exists to
# refuse.
#
# The mtime term is owned by the run, not
# read from the clock. Right after the build, before the identity is read,
# ws_mutant_bind_identity STAMPS the artifact's mtime to a run-unique instant
# no live clock produces — 2001-09-09 plus this shell's pid, whole seconds
# (ws_stamp_artifact) — and then REQUIRES the identity it reads to carry that
# stamp — both through ONE descriptor (ws_stamp_artifact:
# O_NOFOLLOW open, fstat, futimens, fstat), so nothing between the stamp
# and the identity read happens by name. Any later write by anyone sets the
# clock's value and the tuple moves, whatever that clock's granularity.
# Without the stamp the term was
# the filesystem's own mtime, and that is not a reliable key: Linux file
# timestamps come from the COARSE clock (`current_time()` reads
# `ktime_get_coarse_real_ts64`, jiffies granularity — 1 to 4 ms; multigrain
# kernels hand out a fine-grained value only once something has QUERIED the
# timestamp since its last update), so a same-inode, same-size rewrite inside
# one tick keeps the tuple, and no per-file key closes that: a byte-identical
# rebuild defeats a digest, ctime shares the clock, and an inode generation
# is neither reachable from stat(1) nor moved by an in-place rewrite.
# MEASURED on macOS (APFS, nanosecond stamps): 3000 consecutive same-inode
# same-size rewrites coalesced 0 times and 3000 O_TRUNC rewrites 0 times —
# the fine-clock half; and 3000 stamp-then-rewrite trials stored the stamp
# exactly 3000 times and kept it after the rewrite 0 times — the property
# the key rests on, which no clock granularity can change. The stamp
# also covers the recycled-inode case (ext4 hands a freed inode number
# straight back and a rebuilt mutant has the same size): the peer's write
# carries the clock, never this run's stamp. The nanosecond rendering (GNU
# `%.9Y`, BSD `%Fm`) is kept as the fail-closed SHAPE of the key; the stamp
# reads back through it as `<seconds>.000000000`.
#
# What the key ARBITRATES is narrow, deliberately: with every mutant
# built into a run-private root (ws_mutant_root_create) and one run per tool
# directory (ws_run_lock), no peer run can reach a bound name at all. The
# identity is what makes a removal PROVABLE, not what keeps peers apart — a
# same-user write into this run's own root is still detected and refused,
# and a name that has no identity is never swept.
#
# Limits, stated rather than hidden. The identity exists only AFTER the
# build, so an entry registered by a build that never completed has none —
# and an entry with none is NOT removed (the rule at every removal
# site: no identity ⇒ no removal; the path is named, the sweep latches a
# restore failure, and the next run refuses to start while the leftover
# exists). Such an entry is deliberately never swept
# blind: the half-written product of an interrupted build is exactly an
# object this run cannot prove is its own, and a leftover the operator is
# told about is the lesser harm. The identity read and the `rm` are two
# steps, so a swap landing between them is not caught; what IS closed is
# the whole of the re-occupation window — a name this run deleted and something
# else re-occupied before the sweep — plus every swap that lands before a
# removal or before the mutant is RUN (`ws_mutant_verify_identity`, which
# run_matrix.sh calls immediately before executing it; the verify-to-exec
# gap is the same two-step shape and is not claimed closed).
ws_file_identity() {
    # $1 path. dev:inode:size:mtime(ns) of the object AT the name — NO
    # follow (neither call passes -L, and that omission is load-bearing:
    # selftest arm 31(i) plants a symlink at a bound name), so a symlink
    # reports itself and can never carry the bound identity of what it
    # points at. GNU stat, then BSD; on GNU `-f` means FILESYSTEM status and
    # is reached only after `-c` already failed, so its output is never an
    # identity — both are quiet, and the SHAPE below is what is trusted.
    local id
    id=$(stat -c '%d:%i:%s:%.9Y' "$1" 2>/dev/null) \
        || id=$(stat -f '%d:%i:%z:%Fm' "$1" 2>/dev/null) \
        || return 1
    # FAIL CLOSED on shape: digits, colons and dots only, exactly four
    # fields, a sub-second fraction on the last. A stat that rendered no
    # fraction would otherwise bind the weaker dev:inode:size:seconds key the
    # comment above rejects; a spaced or multi-line value would never match
    # its own registry line and send ws_mutant_remove down the blind arm —
    # both silent, both back to removal by name alone. Refused here instead.
    case $id in
        ''|*[![:digit:]:.]*|*:*:*:*:*) return 1 ;;
        [0-9]*:[0-9]*:[0-9]*:[0-9]*.[0-9]*) printf '%s\n' "$id" ;;
        *) return 1 ;;
    esac
}

ws_mutant_identity_of() {
    # $1 registered path. Echoes its bound identity; rc 1 when none is bound.
    # The identity carries no spaces, so the line splits at its FIRST space
    # and the path keeps any spaces of its own.
    local l
    while IFS= read -r l; do
        [ -n "$l" ] || continue
        if [ "${l#* }" = "$1" ]; then
            printf '%s\n' "${l%% *}"
            return 0
        fi
    done <<< "$WS_MUTANT_IDENTS"
    return 1
}

ws_mutant_idents_without() {
    # $1 path. Echoes WS_MUTANT_IDENTS minus every line bound to that path.
    local l
    while IFS= read -r l; do
        [ -n "$l" ] || continue
        [ "${l#* }" = "$1" ] && continue
        printf '%s\n' "$l"
    done <<< "$WS_MUTANT_IDENTS"
}

ws_stamp_artifact() {
    # $1 a regular file this run built. The
    # stamp and the identity read go THROUGH ONE DESCRIPTOR. Gating
    # the name (`-L`/`-f`), stamping BY NAME (os.utime no-follow) and
    # reading the identity BY NAME is racy: a symlink swapped in after the gate
    # has its OWN mtime stamped and its own identity read — and that identity
    # carries the stamp, so the read-back gate binds a link. So the path
    # is opened O_NOFOLLOW (a link at the name is refused by the kernel, not
    # by a check a swap can outrun), the descriptor is fstat'd to be a
    # regular file, stamped with futimens (WS_MUTANT_STAMP_NS, minted by
    # ws_mutants_bind) and fstat'd again for the identity that is ECHOED:
    # `dev:inode:size:seconds.nanoseconds`, the exact rendering
    # ws_file_identity reads by name at the verify and the removal (parity
    # pinned by selftest 31's prelude). Nothing between the stamp and the
    # identity read happens by name, so a write landing in that gap cannot
    # be bound as this run's. A stamp the filesystem does not store exactly
    # (fstat disagrees) is refused: an identity carrying the clock's value
    # would silently be the weaker key.
    if [ -z "$WS_MUTANT_STAMP_NS" ]; then
        echo "error: ws_stamp_artifact: no run stamp minted (call ws_mutants_bind \
first)" >&2
        return 2
    fi
    python3 - "$1" "$WS_MUTANT_STAMP_NS" <<'PYEOF_STAMP'
import os, stat, sys
path = sys.argv[1]
ns = int(sys.argv[2])
try:
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
except OSError as e:
    sys.stderr.write(f"error: ws_stamp_artifact: could not open '{path}' without "
                     f"following a link ({e.strerror}) — refusing to stamp\n")
    sys.exit(2)
st = os.fstat(fd)
if not stat.S_ISREG(st.st_mode):
    sys.stderr.write(f"error: ws_stamp_artifact: the object opened at '{path}' is "
                     f"not a regular file — refusing to stamp\n")
    sys.exit(2)
os.utime(fd, ns=(ns, ns))
st = os.fstat(fd)
if st.st_mtime_ns != ns:
    sys.stderr.write(f"error: ws_stamp_artifact: '{path}' does not carry this run's "
                     f"stamp after it was applied (stored {st.st_mtime_ns}, stamp {ns}) "
                     f"— this filesystem cannot store the stamp; refusing to bind\n")
    sys.exit(2)
print(f"{st.st_dev}:{st.st_ino}:{st.st_size}:{st.st_mtime_ns // 10**9}."
      f"{st.st_mtime_ns % 10**9:09d}")
PYEOF_STAMP
}

ws_bind_precheck() {
    # $1 registered path. The cheap BY-NAME gate: a link or a non-regular
    # object is refused instantly and named (a build produces neither).
    # Not what the bind's identity rests on — the open in
    # ws_stamp_artifact is O_NOFOLLOW and fstat-gated, so a swap landing
    # after this check is refused there. This exists for the diagnosis, and
    # selftest 31(r) overrides it to model exactly that swap.
    if [ -L "$1" ] || [ ! -f "$1" ]; then
        echo "error: ws_mutant_bind_identity: '$1' is not a regular file (it \
is a $(ws_object_kind "$1")) — a build does not produce that; refusing to \
bind" >&2
        return 2
    fi
}

ws_mutant_bind_identity() {
    # $1 a REGISTERED mutant path whose build just completed. STAMPS it and
    # records the identity of the stamped object — both through ONE
    # descriptor (ws_stamp_artifact); a later bind for the same
    # path replaces the earlier one. Refuses an unregistered path — the
    # registry is the only list the removals read, so an identity with no
    # registration would protect nothing — a link or non-regular object
    # (by name, then by the kernel at the open), a stamp that could not be
    # applied or stored, and a rendering that is not the key's shape.
    if ! ws_line_in_list "$1" "$WS_MUTANT_BUILT"; then
        echo "error: ws_mutant_bind_identity: '$1' is not registered by this \
run (call ws_mutant_register first)" >&2
        return 2
    fi
    ws_bind_precheck "$1" || return 2
    # The identity IS the stamped descriptor's (ws_stamp_artifact
    # echoes it); nothing here reads the name again. The shape is still
    # gated — fail closed, exactly as ws_file_identity gates a by-name read —
    # and the stamp field is required, the shell-side pin of what python
    # already checked.
    local id rest
    if ! id=$(ws_stamp_artifact "$1"); then
        echo "error: ws_mutant_bind_identity: could not stamp and identify '$1' \
through one descriptor (see above) — refusing to bind" >&2
        return 2
    fi
    case $id in
        [0-9]*:[0-9]*:[0-9]*:[0-9]*.[0-9]*) ;;
        *)
            echo "error: ws_mutant_bind_identity: the identity rendered for '$1' \
is not dev:inode:size:mtime(ns) ('$id') — refusing to bind" >&2
            return 2
            ;;
    esac
    case $id in
        *:"$WS_MUTANT_STAMP_MTIME") ;;
        *)
            echo "error: ws_mutant_bind_identity: the identity of '$1' does not \
carry this run's stamp (identity $id, stamp $WS_MUTANT_STAMP_MTIME) — refusing \
to bind" >&2
            return 2
            ;;
    esac
    rest=$(ws_mutant_idents_without "$1")
    WS_MUTANT_IDENTS="${rest}
${id} ${1}
"
}

ws_mutant_verify_identity() {
    # $1 registered mutant path, $2 what the caller is about to do with it
    # (for the message). Refuses when no identity is bound, when the object
    # cannot be identified, or when it is no longer the one bound.
    local want now
    if ! want=$(ws_mutant_identity_of "$1"); then
        echo "error: '$1' has no bound identity (its build never completed \
through ws_mutant_bind_identity) — refusing to $2" >&2
        return 2
    fi
    if ! now=$(ws_file_identity "$1" 2>/dev/null); then
        echo "error: '$1' cannot be identified (it is a $(ws_object_kind "$1")) \
— refusing to $2" >&2
        return 2
    fi
    if [ "$now" != "$want" ]; then
        echo "error: '$1' is NOT the artifact this run built (bound $want, \
now $now) — something replaced it behind the pathname; refusing to $2" >&2
        return 2
    fi
    return 0
}

ws_mutant_unregister() {
    # $1 path. Drops every registration line and every identity line for it.
    local l kept=""
    while IFS= read -r l; do
        [ -n "$l" ] || continue
        [ "$l" = "$1" ] && continue
        kept="$kept$l
"
    done <<< "$WS_MUTANT_BUILT"
    WS_MUTANT_BUILT=$kept
    WS_MUTANT_IDENTS=$(ws_mutant_idents_without "$1")
}

ws_mutant_remove() {
    # $1 registered mutant path. THE removal seam for a mutant this run
    # built — run_matrix.sh's per-mutant cleanup goes through here, never
    # through a bare `rm`. Refuses a path this run did not register (this
    # harness never removes a path it did not create); refuses — reported,
    # left in place, rc 2 — an object that no longer carries the identity
    # bound at build time; and refuses, the same way, an entry that carries
    # NO identity (no identity ⇒ no removal) — a half-written
    # product of an interrupted build, or a link or other object that landed
    # at the name before the build completed — because without an identity
    # this run cannot prove the object is its own, and the earlier blind
    # sweep of such an entry is the destructive class this reverses. It
    # UNREGISTERS on success, so a later occupant of the name is reported by
    # the sweep as what it is (not registered by this run) instead of being
    # refused as a swap of an artifact this run no longer holds. The two
    # gates are repeated inline in ws_mutants_restore, which must LATCH a
    # refusal rather than return it and keeps its own removed/unremoved
    # accounting; keep the two in step. rc 0: removed, or already absent.
    if ! ws_line_in_list "$1" "$WS_MUTANT_BUILT"; then
        echo "error: ws_mutant_remove: '$1' was not registered by this run — \
refusing to remove a path this run did not create" >&2
        return 2
    fi
    local want now rm_err
    if [ -e "$1" ] || [ -L "$1" ]; then
        if ! want=$(ws_mutant_identity_of "$1"); then
            echo "error: the object at '$1' was registered by this run but never \
bound to an identity (its build did not complete through \
ws_mutant_bind_identity) — NOT removed: without an identity this run cannot \
prove the object there is its own; delete it by hand once you know what it \
is" >&2
            return 2
        fi
        if ! now=$(ws_file_identity "$1"); then
            # An object this host's stat cannot render as the key
            # (a third dialect, no sub-second field) or that is not a plain
            # object is said so — not accused of being a swap, which would
            # send the operator hunting an intruder.
            echo "error: the mutant prover binary at '$1' cannot be identified \
(it is a $(ws_object_kind "$1"), or this host's stat rendered no \
dev:inode:size:mtime(ns) key) — NOT removed: an object this run cannot \
identify is not this run's to delete" >&2
            return 2
        fi
        if [ "$now" != "$want" ]; then
            echo "error: the mutant prover binary at '$1' is NOT the \
artifact this run built (bound $want, now $now) — NOT removed: something \
replaced it behind the pathname, so this run's verdict for it cannot be \
trusted and the object is not this run's to delete" >&2
            return 2
        fi
        if ! rm_err=$(rm -f "$1" 2>&1); then
            echo "error: a mutant prover binary could NOT be removed: $1 \
($rm_err) — delete it by hand before the next run" >&2
            return 2
        fi
    elif ws_mutant_identity_of "$1" >/dev/null; then
        # Bound at build time and gone: nothing outlives the matrix, so this
        # is not a failure — but it is the mirror image of the swap this seam
        # refuses (something else removed this run's artifact), and it is
        # said out loud.
        echo "warning: the mutant prover binary at '$1' was bound at build \
time and has since VANISHED — something else removed this run's artifact" >&2
    fi
    ws_mutant_unregister "$1"
    return 0
}

# Put the pristine prover back WITHOUT following the pathname: write a temp
# file beside it (same directory, so the rename is a rename and not a copy),
# set the bound mode on the temp, then `mv -f` over the name. Renaming
# replaces whatever object the name currently denotes — a symlink, a FIFO, a
# stale binary — instead of writing through it, and leaves no window in which
# the prover is half-written. Returns non-zero if any step fails; the caller
# turns that into a run failure.
# True iff a restore left the tool directory in a state this module exists to
# prevent — a registered mutant it could not remove, or a production prover it
# could not put back. run_matrix.sh consults this at the very END of its EXIT
# trap, so the failure reaches the exit status without the non-zero return
# that would abort the rest of the trap.
ws_mutants_restore_failed() {
    [ -n "$WS_MUTANT_RESTORE_FAILED" ]
}

ws_mutant_restore_prover() {
    local tmp
    tmp=$(mktemp "${WS_MUTANT_PROD}.restore.XXXXXX") || return 1
    if ! cp "$WS_MUTANT_PRISTINE" "$tmp"; then
        rm -f "$tmp"
        return 1
    fi
    if ! chmod "${WS_MUTANT_MODE:-0755}" "$tmp"; then
        rm -f "$tmp"
        return 1
    fi
    if ! mv -f "$tmp" "$WS_MUTANT_PROD"; then
        rm -f "$tmp"
        return 1
    fi
}

# Exact-LINE membership in a newline-separated list. A `case` glob would
# treat a pathname's own metacharacters as a pattern; this compares strings.
ws_line_in_list() {
    local needle=$1 hay=$2 l
    while IFS= read -r l; do
        [ "$l" = "$needle" ] && return 0
    done <<< "$hay"
    return 1
}

ws_mutants_restore() {
    # Idempotent, and safe from a trap: removes every mutant binary this
    # run registered and puts the production prover back if its bytes
    # changed. Never fails the exit path it runs on — it reports.
    #
    # `set -e` is SUSPENDED for the body, and that is not belt-and-braces:
    # the reporting this function does is itself a command that can fail.
    # MEASURED — with stderr unwritable (`./run_matrix.sh 2>&1 | head`, or a
    # CI `| tee` whose reader exits), a failing `echo … >&2` aborts the
    # function before its `return 0`, which skips the rest of the EXIT trap
    # (the snapshot, the results root, the workspace) AND rewrites the
    # script's exit status. That is the exact triple this function's own
    # contract promises not to cause, reintroduced by the act of reporting.
    # Suspending and restoring is the property-level fix: a reporter added
    # here later cannot re-open it.
    local __cer_restore_e=""
    case $- in
        *e*) __cer_restore_e=1; set +e ;;
    esac
    #
    # RETURNS 0 EXPLICITLY. It is the FIRST statement of run_matrix.sh's
    # EXIT trap, which runs under `set -e`: a non-zero return there aborts
    # the REST of that trap — the snapshot, the results root, the workspace
    # cleanup — and, when it is a trap's last command, rewrites the
    # script's exit status. Every path through this function happens to end
    # 0 today (a false `if` with no `else` returns 0), so the explicit
    # `return` is not what makes that true now; it is what keeps it true
    # under a later edit that ends the function with a test. The PROPERTY,
    # not this line, is what selftest arm 27 pins — and a version of this
    # function ending in a bare test fails that arm.
    local f removed=0 line unremoved="" want now
    while IFS= read -r line; do
        [ -n "$line" ] || continue
        # `-e` follows symlinks, so a registered path that is a BROKEN
        # symlink would be skipped, counted nowhere, and left behind.
        if [ ! -e "$line" ] && [ ! -L "$line" ]; then
            if ws_mutant_identity_of "$line" >/dev/null; then
                echo "warning: the registered mutant prover binary at '$line' \
was bound at build time and has since VANISHED — something else removed this \
run's artifact" >&2
            fi
            continue
        fi
        # A bound identity the name no longer carries is
        # REFUSED — reported and latched as a restore failure, never deleted
        # (the object is not this run's; the gates are ws_mutant_remove's,
        # repeated inline because a sweep must never return non-zero — keep
        # the two in step). An entry with NO bound identity — a
        # build that never completed — is refused and latched the same way,
        # never swept blind: no identity ⇒ no removal.
        if ! want=$(ws_mutant_identity_of "$line"); then
            echo "error: the registered mutant prover binary at '$line' was never \
bound to an identity (its build did not complete through \
ws_mutant_bind_identity) — NOT removed: without an identity this run cannot \
prove the object there is its own; delete it by hand once you know what it \
is" >&2
            WS_MUTANT_RESTORE_FAILED=1
            unremoved="$unremoved$line
"
            continue
        fi
        if ! now=$(ws_file_identity "$line"); then
            echo "error: the registered mutant prover binary at '$line' cannot be \
identified (it is a $(ws_object_kind "$line"), or this host's stat rendered no \
dev:inode:size:mtime(ns) key) — NOT removed: an object this run cannot \
identify is not this run's to delete" >&2
            WS_MUTANT_RESTORE_FAILED=1
            unremoved="$unremoved$line
"
            continue
        fi
        if [ "$now" != "$want" ]; then
            echo "error: the registered mutant prover binary at '$line' is \
NOT the artifact this run built (bound $want, now $now) — NOT removed: \
something replaced it behind the pathname during the run; delete it by hand \
once you know whose it is" >&2
            WS_MUTANT_RESTORE_FAILED=1
            unremoved="$unremoved$line
"
            continue
        fi
        if rm -f "$line" 2>/dev/null; then
            removed=$((removed + 1))
        else
            echo "error: a mutant prover binary could NOT be removed: $line \
— delete it by hand before the next run" >&2
            WS_MUTANT_RESTORE_FAILED=1
            unremoved="$unremoved$line
"
        fi
    done <<< "$WS_MUTANT_BUILT"
    # The entries whose removal FAILED stay REGISTERED; only the removed ones
    # are cleared. Clearing all of them unconditionally made the sweep at the
    # bottom of this function re-report the very same file as "not registered
    # by this run" — contradicting, two messages later, the accurate error
    # just printed above, and telling an operator to look for a build site
    # that skipped its registration when no such site exists.
    WS_MUTANT_BUILT="$unremoved"
    # The identities of the removed entries are left as they are. A stale
    # line is inert for two reasons that are actually true: the only stale
    # line this sweep can leave belongs to a name it has just dropped from
    # the registry, and this sweep is the EXIT trap — nothing runs after it;
    # and `ws_mutant_register` clears a name's identity when the name's next
    # life begins. (`ws_mutant_verify_identity` is the one consumer that does
    # not gate on the registry; its only caller registers immediately before
    # it.) A pruning pass here would be a second guard nothing could
    # verify.
    if [ "$removed" -gt 0 ]; then
        # stderr, like every other diagnostic here. This line is only ever
        # printed when something ABNORMAL happened (the normal path removes
        # each mutant as it is judged and reaches the trap with nothing to
        # do), and run_matrix.sh's stdout is its verdict stream — a restore
        # report is not a verdict.
        echo "restored: removed $removed leftover mutant prover binary(ies) \
— a mutant never outlives the matrix" >&2
    fi
    if [ -n "$WS_MUTANT_PRISTINE" ] && [ -n "$WS_MUTANT_PROD" ]; then
        # The destination is re-validated HERE, immediately before the write,
        # and the write never follows the pathname. The
        # alternative, reproduced: with a symlink planted at the prover pathname before
        # cleanup, `cmp`/`cp`/`chmod` all follow it, so the restore rewrites
        # an UNRELATED file's bytes and sets its mode to the prover's, while
        # the pathname stays a symlink. The bind-time regular-file check
        # cannot help — it was true minutes earlier, which is the whole
        # check-then-use shape.
        #
        # So: a destination that is a symlink or not a regular file is
        # restored UNCONDITIONALLY and `cmp` is skipped entirely (comparing
        # through it is what reads the attacker's target), and the write goes
        # to a temp file in the SAME directory which is then renamed over the
        # pathname. `mv` replaces the NAME, so a symlink or FIFO sitting
        # there is displaced rather than written through, and the replacement
        # is atomic — no window where the prover is half-written.
        local dest_ok=1
        if [ -L "$WS_MUTANT_PROD" ] || [ ! -f "$WS_MUTANT_PROD" ]; then
            dest_ok=""
        fi
        if [ -z "$dest_ok" ] || ! cmp -s "$WS_MUTANT_PROD" "$WS_MUTANT_PRISTINE"; then
            if [ -z "$dest_ok" ]; then
                echo "warning: the production prover pathname '$WS_MUTANT_PROD' \
is not a regular file (it is a $(ws_object_kind "$WS_MUTANT_PROD")) — replacing \
it by rename, without reading or writing through it" >&2
            else
                echo "warning: the production prover '$WS_MUTANT_PROD' is not \
the binary this run built — restoring it from the copy taken before stage 5" >&2
            fi
            if ! ws_mutant_restore_prover; then
                echo "error: the production prover could NOT be restored; \
rebuild it with ./build.sh before trusting it" >&2
                WS_MUTANT_RESTORE_FAILED=1
            fi
        fi
        # OUTSIDE the byte compare, deliberately. `cmp` short-circuits the
        # whole block when the bytes match — so a prover whose MODE was
        # broken during the run (chmod 0644, a umask accident) but whose
        # bytes were not is never touched and nothing is printed: byte-
        # correct, unusable, and reported with silence.
        if [ -f "$WS_MUTANT_PROD" ] && [ ! -L "$WS_MUTANT_PROD" ] &&
                [ "$(ws_object_mode "$WS_MUTANT_PROD")" != \
                  "${WS_MUTANT_MODE:-0755}" ]; then
            if ! chmod "${WS_MUTANT_MODE:-0755}" "$WS_MUTANT_PROD"; then
                echo "error: the production prover's mode could not be set \
back to ${WS_MUTANT_MODE:-0755} — rebuild it with ./build.sh" >&2
                WS_MUTANT_RESTORE_FAILED=1
            fi
        fi
        if [ -e "$WS_MUTANT_PROD" ] && [ ! -x "$WS_MUTANT_PROD" ]; then
            echo "error: the production prover is not executable — rebuild \
it with ./build.sh" >&2
        fi
        f=$WS_MUTANT_PRISTINE
        WS_MUTANT_PRISTINE=""
        WS_MUTANT_PROD=""
        rm -f "$f" || echo "warning: could not remove the pristine prover copy \
(a full copy of the prover is left behind): $f" >&2
    fi
    # The sweep the registry alone cannot do. Everything above removes only
    # what this run REGISTERED; a mutant built at a path nothing recorded —
    # a build site that skipped the registration, a future third caller —
    # would survive in silence with rc 0, which is the module's promise read
    # backwards. Reported, never deleted: the never-remove-what-we-did-not-
    # create rule is what makes the startup refusal trustworthy.
    if [ -n "$WS_MUTANT_DIR" ] || [ -n "$WS_MUTANT_ROOT" ]; then
        local left="" inroot=""
        [ -z "$WS_MUTANT_DIR" ] || left=$(ws_mutant_stale_list "$WS_MUTANT_DIR")
        # This run's private root is swept the same way — a mutant
        # in it that this run did not register is something else's, planted
        # inside a directory only this run should be writing.
        if [ -n "$WS_MUTANT_ROOT" ] && [ -d "$WS_MUTANT_ROOT" ] \
                && [ ! -L "$WS_MUTANT_ROOT" ]; then
            inroot=$(ws_mutant_stale_list "$WS_MUTANT_ROOT")
        fi
        if [ -n "$inroot" ]; then
            left="${left:+$left
}$inroot"
        fi
        # Subtract what this run DID register and could not remove. Those are
        # already reported above, accurately, as removal failures; listing
        # them again here would put a FALSE label on the same pathname.
        if [ -n "$left" ] && [ -n "$WS_MUTANT_BUILT" ]; then
            local swept="" l
            while IFS= read -r l; do
                [ -n "$l" ] || continue
                if ws_line_in_list "$l" "$WS_MUTANT_BUILT"; then
                    continue
                fi
                swept="$swept$l
"
            done <<< "$left"
            left=$swept
        fi
        if [ -n "$left" ]; then
            echo "error: mutant prover binaries survived this run and were not \
registered by it:" >&2
            while IFS= read -r line; do
                [ -n "$line" ] || continue
                echo "  $line" >&2
            done <<< "$left"
            echo "delete them by hand — a mutant must never outlive the matrix" >&2
        fi
        WS_MUTANT_DIR=""
    fi
    # The private root goes with the run. `rmdir`, never `rm -r`:
    # anything still inside was named above (a refused entry, a survivor),
    # and a root that is not empty — or whose name no longer denotes a
    # directory — stays, is named, and fails the run; the next run refuses
    # to start while it exists.
    if [ -n "$WS_MUTANT_ROOT" ]; then
        if [ -e "$WS_MUTANT_ROOT" ] || [ -L "$WS_MUTANT_ROOT" ]; then
            if ! rmdir "$WS_MUTANT_ROOT" 2>/dev/null; then
                echo "error: this run's private mutant root could NOT be removed \
(it is not empty, or the name no longer denotes this run's directory): \
$WS_MUTANT_ROOT — what it still holds is named above; delete it by hand \
before the next run, which refuses to start while it exists" >&2
                WS_MUTANT_RESTORE_FAILED=1
            fi
        fi
        WS_MUTANT_ROOT=""
    fi
    [ -z "$__cer_restore_e" ] || set -e
    return 0
}
