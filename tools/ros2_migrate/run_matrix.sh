#!/usr/bin/env bash
# The cerulion ros2 migrate prover matrix — runs INSIDE the ros2-bench Jazzy
# container (see README.md for the exact docker invocation).
#
# Stages:
#   1. copy the fixture workspace to a scratch colcon ws + git-commit it
#   2. colcon build it with CMAKE_EXPORT_COMPILE_COMMANDS=ON (fixtures
#      compile) and run matrix_runner pre-migration (behavior baseline,
#      rmw_fastrtps_cpp — a non-loaning path for std_msgs/String)
#   3. build the clang tool, analyse every fixture TU, assert the
#      accept/reject matrix (assert_matrix.py — hand-written oracles)
#   4. determinism: two runs of the tool are byte-identical
#   5. variants: each compiled-out prover check flips exactly its fixture
#      from refused to rewritten, then that binary is
#      deleted (the restore)
#   6. [--with-verb-e2e] drive the REAL `cerulion ros2 migrate` verb over
#      the scratch ws: dry-run determinism, --write --yes (commit + patch +
#      auto colcon build of the affected package), patch reversibility
#      (git apply -R --check), post-migration matrix_runner behavior pin,
#      and idempotence (second dry-run proposes nothing)
set -euo pipefail

# The login gate is on in every build, and stage 6 launches the CLI directly,
# so it does not pick up the workspace cargo configuration. This is how this
# repository's own runs pass the gate without an account. The env sweep in
# front of the engine launch removes the ambient GIT_* surface only, so this
# reaches it.
export CERULION_LOGIN_GATE=off

cd "$(dirname "$0")"
HERE=$(pwd)
# The workspace-hardening helpers — the
# hook-proof, object-BOUND git wrapper (ws_git), the binding seams it
# reads (ws_bind_publish / ws_bind_gitdir) and the bound-identity
# re-check for the non-git stages (require_ws_identity) — live in
# matrix_lib.sh so run_matrix_selftest.sh can prove them desk-side
# without a container.
# shellcheck source=matrix_lib.sh
. "$HERE/matrix_lib.sh"

# STRICT argv (the same class as build.sh's argv check): no arguments, or
# exactly `--with-verb-e2e` — a typo'd flag must fail loudly, never
# silently run without stage 6 and still report success.
WITH_VERB_E2E=0
case $# in
    0) ;;
    1)
        [ "$1" = "--with-verb-e2e" ] || {
            echo "usage: ./run_matrix.sh [--with-verb-e2e]" >&2
            exit 2
        }
        WITH_VERB_E2E=1
        ;;
    *)
        echo "usage: ./run_matrix.sh [--with-verb-e2e]" >&2
        exit 2
        ;;
esac

# The check→exec TOCTOU class: ws_git binds every harness
# git call to the VERIFIED workspace + gitdir OBJECTS, and the gitdir
# half needs an fd-traversable /dev/fd (Linux procfs — the container;
# macOS devfs cannot). A host without it would silently degrade to
# resolving .git by name, so refuse up front, before any scratch state
# exists — this script only ever runs in the container.
ws_git_require_fd_binding || exit 2

# Refuse BEFORE any work if a variant prover
# binary is already sitting in this directory. A variant is a prover with a
# proof compiled OUT; one left behind by an aborted run (or built by hand)
# would be reached for as "the tool", and stage 5 would be judged beside
# it. Checked here, before the scratch workspace exists, so the refusal
# costs nothing; the run's OWN variants are bound + restored below.
#
# The build-bind gap: one run per tool
# directory, taken FIRST — before the leftover check, so a `.mutants.*` root
# found there is a dead run's (a live one would be holding this lock), and
# before any work. The variants are private to a run (stage 5, below); the
# production prover and its restore share one pathname here, and two runs
# building it under each other cannot both be right. Held by this shell for
# its whole life and released by the kernel on exit, however the run ends.
ws_run_lock "$HERE" || exit 2
ws_mutants_require_clean "$HERE" || exit 2

# DESTRUCTIVE-PATH GUARD (not an in-directory sentinel,
# which is FORGEABLE: any in-directory token a script checks for, an
# actor can plant, and a marker name planted in a directory
# holding real user files gets them deleted by a recycle). The rule is
# STRUCTURAL: this script never deletes a pre-existing path.
# CERULION_MIGRATE_MATRIX_WS (if set) names a PARENT directory, not the
# workspace; every run creates a FRESH unique workspace under it with
# mktemp -d, records that exact path in $WS, and the ONE rm -rf in this
# script targets only $WS — a path this process provably created, never
# recomputed from the environment, never pre-existing. Nothing to forge.
# Inspection survives: the path is PRINTED, retained on failure, and
# retained on success too when the parent was explicitly chosen (the
# default /tmp parent cleans up on success). Re-runs make fresh subdirs —
# no recycling. Deliberately ABOVE the ROS setup source so the guard is
# drivable on any desk.
WS_PARENT=${CERULION_MIGRATE_MATRIX_WS:-/tmp/cerulion_migrate_matrix}
WS_PARENT_EXPLICIT=${CERULION_MIGRATE_MATRIX_WS:+1}
case "$WS_PARENT" in
    /*) ;;
    *)
        echo "error: CERULION_MIGRATE_MATRIX_WS must be an absolute PARENT \
directory for scratch workspaces (got '$WS_PARENT')" >&2
        exit 2
        ;;
esac
mkdir -p "$WS_PARENT"
# SETUP-SIDE OBJECT BINDING (the deletion-race family's setup
# member: an identity captured from the PATHNAME after
# mktemp lets a same-user replacement landing in that instant have ITS
# identity recorded, and every later operation, cleanup included, adopts
# the attacker's tree). `bind_fresh_ws` binds the shell's cwd to the
# directory OBJECT at the path exactly once, requires it to be EMPTY, and
# captures dev:inode from `.` — the BOUND object, not the name. From that
# point exactly one object is "ours": an EMPTY dir swapped in before the
# bind is indistinguishable from the one mktemp made, and using-then-
# cleaning an empty attacker directory destroys nothing of theirs beyond
# the accepted rmdir residual (see the cleanup trap below); a NON-empty swap is refused
# outright; a swap AFTER the bind changes only what the pathname names —
# later pathname writes (cp/git) would pollute the replacement, but the
# trap's identity check then refuses deletion, so nothing non-empty that
# is not ours is ever destroyed. Data destruction is closed on every
# interleaving; the stated residuals are pollution-not-destruction and
# the empty-dir rmdir.
bind_fresh_ws() {
    # $1: the freshly created path. Echoes the bound identity; exit 2 if
    # the object at the path is not an empty directory.
    if ! cd "$1" 2>/dev/null; then
        echo "error: cannot enter freshly created scratch workspace '$1'" >&2
        return 2
    fi
    if [ -n "$(ls -A .)" ]; then
        cd / || true
        echo "error: freshly created scratch workspace '$1' is not empty \
(another process replaced it) — refusing to use it" >&2
        return 2
    fi
    ws_identity .
}
WS=$(mktemp -d "$WS_PARENT/ws.XXXXXX")
# Command substitution runs bind_fresh_ws in a SUBSHELL: the cd binds the
# object there, and emptiness + identity are read off that ONE bound
# object; the parent shell's cwd never moves.
WS_ID=$(bind_fresh_ws "$WS") || exit 2
# Publish the bound identities for ws_git's own
# un-skippable gate (see matrix_lib.sh) — every harness git call
# re-verifies these before exec'ing git. This goes ONLY through
# ws_bind_publish, which first DISOWNS any WS_BIND_* inherited from the
# caller's environment (loudly) — an ambient WS_BIND_GITDIR_ID used to
# gate this run's own `init` against a gitdir that did not exist yet
# and exit 2 before setup. This run's bindings are the only ones ws_git
# ever sees; the gitdir one is published after the run's own init.
ws_bind_publish "$WS" "$WS_ID"
echo "scratch workspace: $WS"
# TEMPFILE_SECURITY: every intermediate output lives in a
# PRIVATE per-run directory INSIDE the fresh workspace — never a fixed,
# predictable /tmp path a pre-existing symlink could redirect (and
# concurrent runs cannot cross-talk). Cleaned by the same
# descriptor-relative trap as the workspace; retained with it on failure.
MATRIX_TMP="$WS/.matrix-tmp"
mkdir "$MATRIX_TMP"
# DESCRIPTOR-RELATIVE cleanup (supersedes the earlier
# pathname re-check, whose stat→rm seam was still swappable,
# reproduced in review with a FIFO pause at exactly that instant). The trap
# binds the shell's cwd to the directory OBJECT (`cd "$WS"` — pathname
# swaps after this are irrelevant to fd-relative operations), identity-
# checks `.` (belt and braces: catches a swap that beat the cd), deletes
# the CONTENTS fd-relative (`find . -mindepth 1 -delete` — fts walks
# from the cwd, so it empties only the bound original, whatever the
# pathname now names), and finishes with the ONE remaining pathname
# operation: `rmdir "$WS"`, which removes exclusively an EMPTY directory
# — a swapped-in replacement tree survives with ENOTEMPTY, reported and
# retained. The guarantee is scoped to objects OUTSIDE this script's own
# scratch workspace; the two residuals, precisely: (a)
# contents another process plants INSIDE the workspace between the
# identity check and the deletion are removed with it — that is what
# deleting one's own temp directory means, the same semantics as every
# mktemp consumer and /tmp cleaner (data placed inside another process's
# scratch dir has no integrity expectation); (b) an attacker's *empty*
# directory at the pathname can be rmdir'd — not a data-destruction
# primitive. Both explicitly outside this test tooling's threat model.
# The snapshot root (a private mktemp -d this run created) is
# deleted by the same descriptor-relative discipline: bind the cwd to the
# object, identity-check `.`, delete the contents fd-relative, rmdir.
cleanup_snapshot() {
    [ -n "${SNAP_ROOT:-}" ] || return 0
    if ! cd "$SNAP_ROOT" 2>/dev/null; then
        return 0
    fi
    if [ "$(ws_identity . 2>/dev/null || true)" != "${SNAP_ID:-}" ]; then
        cd / || true
        echo "snapshot root was replaced by another process — refusing to \
delete it; retained: $SNAP_ROOT" >&2
        return 0
    fi
    ws_snapshot_unseal .
    find . -mindepth 1 -delete || true
    cd / || true
    rmdir "$SNAP_ROOT" 2>/dev/null || true
}
cleanup_results() {
    # The private results root, same discipline as the snapshot.
    [ -n "${RESULTS_ROOT:-}" ] || return 0
    if ! cd "$RESULTS_ROOT" 2>/dev/null; then
        return 0
    fi
    if [ "$(ws_identity . 2>/dev/null || true)" != "${RESULTS_ID:-}" ]; then
        cd / || true
        echo "results root was replaced by another process — refusing to \
delete it; retained: $RESULTS_ROOT" >&2
        return 0
    fi
    ws_snapshot_unseal .
    find . -mindepth 1 -delete || true
    cd / || true
    rmdir "$RESULTS_ROOT" 2>/dev/null || true
}
cleanup_ws() {
    code=$?
    # `set -e` is SUSPENDED for the WHOLE trap, and stays suspended.
    #
    # This script runs under `set -euo pipefail`, so ANY failing command in
    # this trap aborts the rest of it. Reporting is a failing command when
    # stdout or stderr is unwritable — `./run_matrix.sh 2>&1 | head`, a CI
    # `| tee` whose reader exited — and every step here reports. Guarding
    # it inside `ws_mutants_restore` and nowhere else would leave the
    # same hole in FOUR more places in the same trap: `cleanup_snapshot`'s
    # and `cleanup_results`' identity-mismatch warnings, `cleanup_ws_
    # workspace`'s retention lines, and — worst — the promotion at the
    # bottom, whose own `echo` sits BEFORE its `exit 2` and so could abort
    # the trap into exiting 0 with a corrupted production prover.
    # The property is what actually needs holding, so it is
    # held once, here, where a reporter added to any step later cannot
    # re-open it.
    #
    # NOT restored before the promotion, deliberately: restoring `set -e`
    # anywhere above that `exit 2` would re-open exactly this hole for the
    # promotion's own report. This function only ever runs as the EXIT trap,
    # so the shell is terminating either way.
    case $- in
        *e*) set +e ;;
    esac
    # Runs FIRST, and on every path a trap can run
    # on — a variant prover must not outlive the matrix that built it, and
    # the production prover a later run reaches for must be the one this
    # run built. The EXIT trap is the whole coverage: bash runs it when the
    # shell is terminated by a fatal signal too. MEASURED on this repo's bash
    # for SIGTERM, SIGINT and SIGHUP; selftest arm 27 drives SIGTERM as the
    # standing guard, so a shell where that stops holding fails the arm
    # rather than going quiet. SIGINT is deliberately not driven by an
    # exit-status oracle: measured, bash DEFERS a signal it has no trap for
    # until the running foreground command finishes and then exits 0, so the
    # valid assertion there is "the trap ran" — which the SIGTERM arm
    # already establishes. Nothing covers SIGKILL: that, and a variant built
    # by hand, are what the startup refusal above is for.
    ws_mutants_restore
    cleanup_snapshot
    cleanup_results
    cleanup_ws_workspace
    # LAST, and after every other cleanup has run. A restore that could not
    # put the tool directory back is not a warning: an otherwise-green matrix
    # would exit 0 leaving a corrupted or unusable production prover, which
    # is "a variant prover outlives the matrix" by another route — this
    # script's own contract. The failure is latched rather than returned
    # because a non-zero return from the restore would abort the rest of THIS
    # trap, which is the defect the restore was written to stop causing.
    if ws_mutants_restore_failed; then
        echo "run_matrix: the tool directory was NOT restored (see the errors \
above) — failing the run so a later one does not trust the prover" >&2
        exit 2
    fi
}

cleanup_ws_workspace() {
    # A failed restore fails the run — the
    # promotion at the end of cleanup_ws exits 2 for it — so this run's
    # scratch workspace must be retained exactly as any other failure's is.
    # It is consulted HERE rather than at the promotion because the
    # promotion has to stay LAST (it is the only thing in the trap that can
    # `exit`, and running it earlier would skip the remaining cleanups —
    # the defect sub-arm (a2) exists to stop), and by the time it runs this
    # function has already deleted the tree a human would read to find out
    # what the matrix left behind. Interaction introduced by the restore
    # fix itself: before it, a body that succeeded could not end in a
    # failing run.
    if ws_mutants_restore_failed; then
        echo "scratch workspace retained because the tool directory could \
NOT be restored: $WS"
        return
    fi
    if [ "$code" -eq 0 ] && [ -z "$WS_PARENT_EXPLICIT" ]; then
        if ! cd "$WS" 2>/dev/null; then
            echo "scratch workspace could not be entered for cleanup — \
left as-is: $WS" >&2
            return
        fi
        if [ "$(ws_identity . 2>/dev/null || true)" != "$WS_ID" ]; then
            cd / || true
            echo "scratch workspace was replaced by another process — \
refusing to delete it; retained: $WS" >&2
            return
        fi
        # Test seam (verification only): lets the harness pause the trap
        # at the exact identity-check→delete instant to drive the
        # swap-after-check arm deterministically. Inert unless set.
        if [ -n "${CERULION_MATRIX_CLEANUP_PAUSE_FIFO:-}" ]; then
            read -r _ < "$CERULION_MATRIX_CLEANUP_PAUSE_FIFO" || true
        fi
        find . -mindepth 1 -delete || true
        cd / || true
        if ! rmdir "$WS" 2>/dev/null; then
            echo "scratch workspace pathname no longer names this run's \
(now empty) directory — left as-is: $WS" >&2
        fi
    else
        echo "scratch workspace retained: $WS"
    fi
}
trap cleanup_ws EXIT

# ROS setup scripts reference unset variables — relax `set -u` around the
# source only.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
# EXECUTION TRUST BOUNDARY (hostile-workspace hook
# execution): after the bind, the harness EXECUTES content living under
# the workspace pathname — git (hooks + config), colcon (builds the
# sources), and the built matrix_runner binary. Structurally trusted is
# only what lives OUTSIDE the workspace: /opt/ros (the container image),
# $HERE (this repo's tools tree: build.sh, the clang tool,
# assert_matrix.py), and /work (the mounted repo). Everything under $WS
# is swappable by a same-user writer, so (a) every harness git call goes
# through ws_git — command-line config outranks a replaced .git/hooks or
# .git/config, so the execution channels cannot be re-enabled from
# inside the tree — and (b) require_ws_identity re-verifies the bound
# dev:inode before every stage that executes workspace content. The
# stated residual for the NON-git stages is the check→exec instant
# (colcon, the cargo apply and matrix_runner take pathnames a shell
# cannot bind); for git itself it is CLOSED — ws_git binds each
# call to the verified workspace object (cwd-bound) and gitdir object
# (fd-bound, `--git-dir=/dev/fd/N`), so a pathname swap after its check
# can no longer route git into a replacement tree (matrix_lib.sh has
# the mechanism, the measurements and the residuals). The
# pollution-not-destruction residuals are those stated above (the
# cleanup trap and the setup-side binding).
# Stage 6's `cerulion ros2 migrate --write`
# runs the ENGINE's git with hooks ON deliberately — honoring the user's
# own hooks on the migration commit is product behavior the engine e2e
# suite pins — so the identity re-check in front of that stage is its
# guard.
# The boundary extends to the GITDIR and to FILTERS: clean/
# smudge commands resolve exclusively from config, ws_git nulls the
# system + global layers, and the remaining sources all live inside
# .git — whose identity is bound at our own `init` (GITDIR_ID) and
# re-verified with the workspace before every executing stage. Apply,
# build and execute within one stage each get their own re-check.
require_ws_path_identity "$WS" "$WS_ID" "the stage-1 fixture baseline" || exit 2
cp -R fixtures/src "$WS/src"
# ws_git runs INSIDE the bound workspace object — callers pass
# no -C (the wrapper refuses one: a pathname there would re-target git
# past the binding).
ws_git init -q
# Bind the GITDIR identity the instant we mint it — with
# system/global config nulled in ws_git, .git is the only place a
# clean/smudge filter command could come from, so every post-init stage
# re-verifies it beside the workspace (require_ws_and_gitdir). It is
# read from INSIDE the bound workspace object (ws_bind_gitdir), never
# from the pathname — the setup-side binding class above — and published to
# ws_git in the same step.
ws_bind_gitdir || exit 2
GITDIR_ID=$WS_BIND_GITDIR_ID
ws_git add -A
ws_git -c user.email=matrix@example.invalid -c user.name=matrix \
    commit -qm "fixture baseline"

echo "== stage 2: fixture workspace builds =="
require_ws_and_gitdir "$WS" "$WS_ID" "$GITDIR_ID" "the stage-2 fixture build" || exit 2
(cd "$WS" && colcon build --packages-select migrate_fixture_pkg \
    --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON)
echo "== stage 2b: pre-migration behavior baseline (rmw_fastrtps_cpp) =="
require_ws_and_gitdir "$WS" "$WS_ID" "$GITDIR_ID" "the stage-2b behavior baseline" || exit 2
(cd "$WS" && RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ./build/migrate_fixture_pkg/matrix_runner)

# Stages 3-5 COLLECT failures instead of stopping at the first one, so a
# single run reports everything; the final verdict (and stage 6) still
# gates on all of them.
FAILURES=0
flunk() {
    echo "RUN_MATRIX FAIL: $1"
    FAILURES=$((FAILURES + 1))
}

echo "== stage 3: tool build + accept/reject matrix =="
./build.sh
TOOL="$HERE/cerulion-ros2-migrate-clang"
# Bind the prover just built, so the restore
# can put THIS binary back if anything replaces it, and so every variant
# stage 5 builds is registered against it.
ws_mutants_bind "$HERE" "$TOOL" || exit 2
# The build-bind gap: every variant is built
# INTO a run-private root under $HERE (mktemp -d, 0700) and identity-bound
# there. Built at the shared name, a variant sat at a pathname a concurrent
# run could rewrite between its build and its bind — this run would then
# bind the OTHER run's artifact as its own and, every later check passing,
# delete it at cleanup. A name no other run can write has no such gap. The
# root is removed by the restore (rmdir — whatever it still holds is named).
ws_mutant_root_create || exit 2
CCDIR="$WS/build/migrate_fixture_pkg"
OUTDIR="$WS/analysis"
mkdir -p "$OUTDIR"
# The prover is pointed at two descendant
# directories — the sources and the compile database — which the
# workspace + gitdir binds did not cover, so a replacement of either
# changed the analyzed inputs undetected. Both are bound here (from
# inside the bound workspace, gated to be real directories) and
# re-verified — beside the workspace and gitdir — immediately before
# EVERY analyzer execution through `analyze`, which is the only way a
# prover binary is ever invoked in this script (pinned by the selftest).
SRC_ID=$(ws_bind_descendant src) || exit 2
CCDIR_ID=$(ws_bind_descendant build/migrate_fixture_pkg) || exit 2
# The inputs are bound by content as well — the
# prover reads files by pathname, so a replaced source or compile db
# leaves the directory identities intact. The digests are captured here,
# BEFORE any analysis, and re-verified after every analysis stage
# (require_analysis_digest); a substitution landing inside a stage fails
# the matrix rather than passing a verdict over substituted inputs.
SRC_DIGEST=$(ws_tree_digest "$WS/src") || exit 2
CCDB_DIGEST=$(ws_file_sha256 "$CCDIR/compile_commands.json") || exit 2
require_analysis_inputs() {
    # $1 stage label.
    require_ws_and_gitdir "$WS" "$WS_ID" "$GITDIR_ID" "$1" || return 2
    require_ws_descendant src "$SRC_ID" "$1" || return 2
    require_ws_descendant build/migrate_fixture_pkg "$CCDIR_ID" "$1" || return 2
}
require_analysis_digest() {
    # $1 stage label. The content bind: sources + compile db unchanged.
    require_ws_tree_digest "$WS/src" "$SRC_DIGEST" "$1" || return 2
    require_ws_file_digest "$CCDIR/compile_commands.json" "$CCDB_DIGEST" "$1" || return 2
    # The SNAPSHOT re-digests itself too — the prover reads it
    # by pathname, so an in-snapshot substitution must fail the stage.
    require_ws_snapshot_intact "$SNAP_ROOT" "$SRC_DIGEST" "$SNAP_CCDB_DIGEST" "$1" || return 2
}
# The prover analyzes an immutable
# verified copy of the approved inputs, never the live tree — see
# matrix_lib.sh (ws_snapshot_inputs). The snapshot root is fresh + private
# (mktemp -d, 0700) and identity-bound; the copy is re-digested against
# the digests approved above; the compile db copy's source paths are
# rewritten to the snapshot; every prover run goes through
# ws_analyze_snapshot; the outputs are mapped back to the live paths for
# the consumers that act on the live tree. Sealed read-only before the
# first prover run and re-digested itself after every stage.
# Deleted by the EXIT trap (unsealed first).
SNAP_ROOT=$(mktemp -d "${TMPDIR:-/tmp}/cerulion_migrate_snap.XXXXXX") || exit 2
SNAP_ID=$(ws_identity "$SNAP_ROOT")
ws_snapshot_inputs "$WS/src" "$SRC_DIGEST" "$CCDIR/compile_commands.json" "$CCDB_DIGEST" "$SNAP_ROOT" || exit 2
ws_rewrite_ccdb_paths "$SNAP_ROOT/ccdb/compile_commands.json" "$WS/src" "$SNAP_ROOT/src" || exit 2
# The rewritten compile db is the snapshot's own approved
# bytes from here on; then the whole snapshot is sealed read-only (files
# AND directories, root 0500) BEFORE any prover runs. Prevention for an
# unprivileged writer; detection (seal re-check + snapshot re-digest in
# every stage) for a same-UID one — see ws_snapshot_seal.
SNAP_CCDB_DIGEST=$(ws_file_sha256 "$SNAP_ROOT/ccdb/compile_commands.json") || exit 2
ws_snapshot_seal "$SNAP_ROOT" || exit 2
echo "analysis inputs snapshotted (verified copy, sealed read-only): $SNAP_ROOT"
# Prover output that went to an
# operator-visible dir ($WS/analysis) and was read back later by
# pathname would let assert_matrix.py and the 3b apply consume whatever sat
# there, so a same-user process that replaced an oracle-compatible JSON in
# that window would have its FORGED edits applied. Results go ONLY into a
# fresh private results root (mktemp -d; 0700 per stage), written
# create-new (ws_capture_result), sealed read-only the moment the stage's
# prover runs end (ws_results_seal), digested over the EXACT expected
# file set (ws_results_digest — extra or missing refuses), and every
# consumer re-verifies that digest immediately before AND after reading,
# reading ONLY from the sealed dir (consume_results). $OUTDIR is an
# EXPORT written from the sealed dir for humans — nothing reads it. A
# same-UID writer can chmod the seal away: against that writer this is
# DETECTION plus the fresh private root, not prevention.
RESULTS_ROOT=$(mktemp -d "${TMPDIR:-/tmp}/cerulion_migrate_results.XXXXXX") || exit 2
RESULTS_ID=$(ws_identity "$RESULTS_ROOT")
mkdir -m 0700 "$RESULTS_ROOT/stage3" "$RESULTS_ROOT/stage4" "$RESULTS_ROOT/stage5" || exit 2
consume_results() {
    # $1 label, $2 sealed results dir, $3 its digest, $4 its expected
    # set, then the consumer command. The results root's identity, the
    # seal, the set and the digest are re-verified immediately before the
    # read and again after it (a swap DURING the read is caught too). A
    # verification failure is a harness refusal (exit 2), never a matrix
    # verdict.
    local label=$1 dir=$2 digest=$3 set=$4 rc
    shift 4
    require_ws_path_identity "$RESULTS_ROOT" "$RESULTS_ID" "$label (results root)" || exit 2
    require_ws_results_intact "$dir" "$digest" "$set" "$label, pre-read" || exit 2
    "$@"
    rc=$?
    require_ws_results_intact "$dir" "$digest" "$set" "$label, post-read" || exit 2
    return "$rc"
}
analyze() {
    # $1 stage label, $2 prover binary, $3 TU relative to src. The live
    # tree is re-verified (the cross-check layer), the snapshot root re-bound,
    # and the prover pointed at the SNAPSHOT only.
    local label=$1 prover=$2 rel=$3
    require_analysis_inputs "$label" || return 2
    require_ws_path_identity "$SNAP_ROOT" "$SNAP_ID" "$label (snapshot root)" || return 2
    require_ws_snapshot_sealed "$SNAP_ROOT" "$label (snapshot seal)" || return 2
    ws_analyze_snapshot "$SNAP_ROOT" "$prover" "$rel"
}
STAGE3_SET=$(for f in "$WS"/src/migrate_fixture_pkg/src/*.cpp; do printf '%s.json\n' "$(basename "$f")"; done | LC_ALL=C sort)
for f in "$WS"/src/migrate_fixture_pkg/src/*.cpp; do
    n="$(basename "$f").json"
    ws_capture_result "$RESULTS_ROOT/stage3/$n" \
        analyze "the stage-3 analysis of $(basename "$f")" "$TOOL" "migrate_fixture_pkg/src/$(basename "$f")" || exit 2
    ws_map_snapshot_paths "$RESULTS_ROOT/stage3/$n" "$SNAP_ROOT/src" "$WS/src" || exit 2
done
ws_results_seal "$RESULTS_ROOT/stage3" || exit 2
STAGE3_DIGEST=$(ws_results_digest "$RESULTS_ROOT/stage3" "$STAGE3_SET") || exit 2
require_analysis_digest "stage 3 (post-analysis)" || exit 2
# Export for humans, FROM the sealed dir — no consumer reads $OUTDIR.
cp -f "$RESULTS_ROOT"/stage3/*.json "$OUTDIR"/ || exit 2
echo "stage-3 analysis exported for inspection (nothing reads it): $OUTDIR"
consume_results "the stage-3 matrix assertion" "$RESULTS_ROOT/stage3" "$STAGE3_DIGEST" "$STAGE3_SET" \
    python3 assert_matrix.py "$RESULTS_ROOT/stage3" || flunk "accept/reject matrix (stage 3)"

echo "== stage 3b: applied bytes BUILD and BEHAVE (the verb's own apply path) =="
# A measured lesson: the matrix's kind/reason/substring oracles can
# pass on rewrite bytes that do not compile (the doubled-arrow class). Apply
# the analysis to the scratch sources through the SAME Rust apply_edits the
# verb uses (cerulion_cli_engine's ros2_migrate_apply example), rebuild the
# fixture package, and re-run the behavior pin — then restore the sources
# for the later stages. Needs the repo mounted at /work (cargo builds the
# engine example on first run).
require_analysis_inputs "the stage-3b apply + rebuild" || exit 2
STAGE3_ARGS=()
while IFS= read -r n; do
    if [ -n "$n" ]; then STAGE3_ARGS+=("$RESULTS_ROOT/stage3/$n"); fi
done <<< "$STAGE3_SET"
apply_stage3() {
    (cd /work && cargo run -q -p cerulion_cli_engine --example ros2_migrate_apply \
        -- "$WS/src" "${STAGE3_ARGS[@]}")
}
if consume_results "the stage-3b apply" "$RESULTS_ROOT/stage3" "$STAGE3_DIGEST" "$STAGE3_SET" apply_stage3; then
    # Apply, build and execute are THREE executing
    # operations — the identity is re-verified before each, not once for
    # the stage (a swap during the cargo apply must not reach colcon).
    require_ws_and_gitdir "$WS" "$WS_ID" "$GITDIR_ID" "the stage-3b rebuild" || exit 2
    if (cd "$WS" && colcon build --packages-select migrate_fixture_pkg); then
        echo "rewritten fixtures build OK"
        require_ws_and_gitdir "$WS" "$WS_ID" "$GITDIR_ID" "the stage-3b behavior pin" || exit 2
        (cd "$WS" && RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
            ./build/migrate_fixture_pkg/matrix_runner) \
            || flunk "rewritten fixtures changed behavior (stage 3b)"
    else
        flunk "rewritten fixtures do not BUILD (stage 3b)"
    fi
else
    flunk "edit application failed (stage 3b)"
fi
ws_git checkout -- src
echo "stage 3b sources restored"
# The restore must return the EXACT bound content — the same digest.
require_analysis_digest "stage 3b (post-restore)" || exit 2

echo "== stage 4: determinism =="
ws_capture_result "$RESULTS_ROOT/stage4/det_a.json" \
    analyze "the stage-4 determinism run A" "$TOOL" migrate_fixture_pkg/src/safe_unique.cpp || exit 2
ws_capture_result "$RESULTS_ROOT/stage4/det_b.json" \
    analyze "the stage-4 determinism run B" "$TOOL" migrate_fixture_pkg/src/safe_unique.cpp || exit 2
ws_results_seal "$RESULTS_ROOT/stage4" || exit 2
STAGE4_SET=$'det_a.json\ndet_b.json'
STAGE4_DIGEST=$(ws_results_digest "$RESULTS_ROOT/stage4" "$STAGE4_SET") || exit 2
require_analysis_digest "stage 4 (post-analysis)" || exit 2
if consume_results "the stage-4 determinism compare" "$RESULTS_ROOT/stage4" "$STAGE4_DIGEST" "$STAGE4_SET" \
        cmp "$RESULTS_ROOT/stage4/det_a.json" "$RESULTS_ROOT/stage4/det_b.json"; then
    echo "determinism OK"
else
    flunk "tool output not byte-deterministic (stage 4)"
fi

echo "== stage 5: prover mutants (run + restore) =="
# NOTE: called in an `if !` context, so `set -e` is suppressed inside —
# every step carries its own explicit failure return.
run_mutant() {
    # Two modes: "rewrite" (default) — the variant must flip the fixture
    # from refused to REWRITTEN; "silent" — the variant must SILENCE the
    # fixture entirely (no rewrites AND no candidates: the
    # FALSE_SKIP shape, where dropping the proof makes the site vanish
    # from the report instead of flipping to a rewrite).
    local name=$1 fixture=$2 mode=${3:-rewrite}
    local lower
    lower=$(printf '%s' "$name" | tr '[:upper:]' '[:lower:]')
    local mut="$WS_MUTANT_ROOT/cerulion-ros2-migrate-clang-mutant-$lower"
    # Registered BEFORE the build, so a binary
    # an interrupted build left half-written is NAMED by the sweep (and
    # never removed blind — see the bind below). The per-variant removal
    # below (through ws_mutant_remove) still runs on the normal
    # path — this is what covers the paths that never reach it.
    # `|| return 2` is load-bearing, not defensive: every call site is
    # `if ! run_mutant …`, and bash disables `set -e` for the whole dynamic
    # extent of a function invoked in a condition — so a bare call would
    # print the refusal and then build the variant anyway, unregistered,
    # which is exactly the leftover this registration exists to prevent.
    ws_mutant_register "$mut" || return 2
    ./build.sh --out-dir "$WS_MUTANT_ROOT" --mutant "$name" || {
        echo "mutant $name failed to BUILD (harness failure)"
        return 2
    }
    # The registry binds the ARTIFACT, not the
    # pathname. Its identity is captured HERE — right after the build, the
    # earliest it exists — verified again immediately before the variant is
    # RUN, and every removal below goes through ws_mutant_remove, which
    # refuses a name that no longer carries it. A bind that FAILS
    # leaves the object in place, named — no identity ⇒ no removal, at this
    # site as at every other: without an identity this run cannot prove the
    # object at the name is its own, and a blind removal
    # here is the destructive class. The registration stays, so the EXIT-trap
    # sweep names it again, latches a restore failure, and the next run
    # refuses to start while it exists. (A build that FAILS leaves the same
    # registered, unbound entry; a compiler that produced nothing leaves
    # nothing at the name to report.)
    ws_mutant_bind_identity "$mut" || {
        echo "mutant $name could not be BOUND — '$mut' is left in place, NOT \
removed: without an identity this run cannot prove the object there is its \
own (harness failure)"
        return 2
    }
    local rdir="$RESULTS_ROOT/stage5/$lower" rdigest
    mkdir -m 0700 "$rdir" || { ws_mutant_remove "$mut"; return 2; }
    # Deliberately NOT followed by a removal on refusal: an object that is
    # not this run's artifact is not this run's to delete. The EXIT trap
    # refuses it the same way and fails the run.
    ws_mutant_verify_identity "$mut" "run the stage-5 mutant $name" || return 2
    ws_capture_result "$rdir/mut.json" \
        analyze "the stage-5 mutant $name" "$mut" "migrate_fixture_pkg/src/$fixture" || {
        echo "mutant $name failed to RUN (harness failure)"
        ws_mutant_remove "$mut"
        return 2
    }
    ws_results_seal "$rdir" || { ws_mutant_remove "$mut"; return 2; }
    rdigest=$(ws_results_digest "$rdir" "mut.json") || { ws_mutant_remove "$mut"; return 2; }
    judge_mutant() {
        MUTANT_RESULT="$rdir/mut.json" python3 - "$fixture" "$name" "$mode" <<'PYEOF'
import json, os, sys
doc = json.load(open(os.environ["MUTANT_RESULT"]))
fixture, name, mode = sys.argv[1], sys.argv[2], sys.argv[3]
if mode == "silent":
    if doc.get("rewrites") or doc.get("candidates"):
        print(f"MUTANT {name} NOT KILLED: {fixture} still reports — the "
              "proof under mutation is not load-bearing (or the fixture "
              "drifted)")
        sys.exit(1)
    print(f"mutant {name} killed by {fixture}: dropping the proof makes "
          "the site vanish from the report (the FALSE_SKIP shape)")
    sys.exit(0)
if mode == "shadow":
    # The kill is the COLLIDING mint — with the AST name scan
    # dropped, the site still rewrites but names its local `loaned`,
    # shadowing the macro-referenced member the raw body text hides.
    import re
    repl = "".join(e.get("replacement", "")
                   for r in doc.get("rewrites", [])
                   for e in r.get("edits", []))
    if not re.search(r"\bloaned\b", repl):
        print(f"MUTANT {name} NOT KILLED: {fixture} still avoids the "
              "colliding name — the scan under mutation is not "
              "load-bearing (or the fixture drifted)")
        sys.exit(1)
    print(f"mutant {name} killed by {fixture}: without the AST name scan "
          "the mint collides with the macro-referenced member")
    sys.exit(0)
if not doc.get("rewrites"):
    print(f"MUTANT {name} NOT KILLED: {fixture} still refused — the check "
          "under mutation is not load-bearing (or the fixture drifted)")
    sys.exit(1)
print(f"mutant {name} killed by {fixture}: the dropped check is what "
      "refused this fixture")
PYEOF
    }
    consume_results "the stage-5 judgement of $name" "$rdir" "$rdigest" "mut.json" judge_mutant \
        || { ws_mutant_remove "$mut"; return 1; }
    # restore: variants never outlive the matrix — and a removal REFUSED here
    # (the name no longer carries the artifact this run built and judged)
    # fails the variant, since the verdict above was taken over an object
    # something else has since replaced.
    ws_mutant_remove "$mut" || return 2
}
if ! run_mutant DROP_ESCAPE_CHECK unsafe_escape.cpp; then
    flunk "mutant DROP_ESCAPE_CHECK (stage 5)"
fi
if ! run_mutant DROP_USE_AFTER_PUBLISH unsafe_reuse_after_move.cpp; then
    flunk "mutant DROP_USE_AFTER_PUBLISH (stage 5)"
fi
if ! run_mutant DROP_PUBLISHER_TRIVIALITY unsafe_wrong_publisher.cpp; then
    flunk "mutant DROP_PUBLISHER_TRIVIALITY (stage 5)"
fi
# DROP_ARROW_IDENTITY was retired: the earlier
# arrow-identity refusal is provably subsumed by the
# exact-publisher gate (proof at the prover's strip site), so the check —
# and this variant — were deleted; unsafe_stateful_arrow.cpp keeps its
# matrix row under the gate's reason, guarded by DROP_EXACT_PUBLISHER.
if ! run_mutant DROP_CTOR_WRITTEN_GUARD unsafe_written_ctor.cpp; then
    flunk "mutant DROP_CTOR_WRITTEN_GUARD (stage 5)"
fi
# matrix12 retarget: unsafe_fake_loan's cross-chain impostor is ALSO
# refused by the TARGET proof, so it can no longer isolate the
# class proof — unsafe_derived_loan (same chain, derived hider)
# is the shape only the class proof refuses.
if ! run_mutant DROP_LOAN_PROOF unsafe_derived_loan.cpp silent; then
    flunk "mutant DROP_LOAN_PROOF (stage 5)"
fi
if ! run_mutant DROP_CALL_MUTATION unsafe_swap_publisher.cpp; then
    flunk "mutant DROP_CALL_MUTATION (stage 5)"
fi
if ! run_mutant DROP_LOAN_TARGET_PROOF unsafe_cross_loan.cpp silent; then
    flunk "mutant DROP_LOAN_TARGET_PROOF (stage 5)"
fi
if ! run_mutant DROP_BODY_TEXT_GUARD unsafe_macro_body.cpp; then
    flunk "mutant DROP_BODY_TEXT_GUARD (stage 5)"
fi
if ! run_mutant DROP_EXACT_PUBLISHER unsafe_derived_publisher.cpp; then
    flunk "mutant DROP_EXACT_PUBLISHER (stage 5)"
fi
if ! run_mutant DROP_AST_NAME_SCAN safe_macro_member_loaned.cpp shadow; then
    flunk "mutant DROP_AST_NAME_SCAN (stage 5)"
fi
# The preprocessor macro scan — an object-like `#define loaned`
# active in the TU; with the scan compiled out the mint collides.
if ! run_mutant DROP_MACRO_NAME_SCAN safe_macro_loaned_defined.cpp shadow; then
    flunk "mutant DROP_MACRO_NAME_SCAN (stage 5)"
fi
if ! run_mutant DROP_CONTROL_FLOW_WALK unsafe_unbraced_publish.cpp; then
    flunk "mutant DROP_CONTROL_FLOW_WALK (stage 5)"
fi
if ! run_mutant DROP_ALIAS_SCAN unsafe_aliased_publisher.cpp; then
    flunk "mutant DROP_ALIAS_SCAN (stage 5)"
fi
# The publisher's name re-declared between the
# two edits — the borrow is spliced above the shadow and resolves to a
# different publisher than the publish. This variant compiles out the WHOLE
# block, so the fixture's TEN refusals flip to rewrites: seven shadow
# mechanisms, two type-headed qualifiers and one using-directive. The accept
# envelopes (safe_same_name_other_scope.cpp, safe_qualified_publisher.cpp)
# are asserted by the stage-3 matrix and are what stop a blanket refusal
# passing this.
#
# What THIS variant asserts is narrower than that sentence, and the stage-3
# matrix is what covers the difference: `run_mutant`'s rewrite judge kills on
# the fixture producing ANY rewrite, so a regression that broke only one of
# the ten arms would still read as killed. The per-arm gate is stage 3's
# EXPECTED multiset, not this line.
if ! run_mutant DROP_PUBLISHER_SHADOW unsafe_shadowed_publisher.cpp; then
    flunk "mutant DROP_PUBLISHER_SHADOW (stage 5)"
fi
# The second part of that guard: a template argument in the publisher
# expression. Its names are resolved by unqualified lookup at the publish
# site, so they are not reached by the member/qualified lookup that makes a
# chain's tail immune — and the shadow root analysis above models neither. The
# belt is taken on the written TEXT, so it is total over the class; the
# variant compiles it out and both of the fixture's sites flip to rewrites.
# safe_qualified_publisher.cpp is the accept control: the same qualified and
# namespace-scope shapes WITHOUT template arguments must still rewrite.
if ! run_mutant DROP_PUBLISHER_TEMPLATE_ARGS unsafe_template_argument_publisher.cpp; then
    flunk "mutant DROP_PUBLISHER_TEMPLATE_ARGS (stage 5)"
fi

require_analysis_digest "stage 5 (post-mutants)" || exit 2

if [ "$FAILURES" -gt 0 ]; then
    echo "run_matrix: $FAILURES stage(s) FAILED (see RUN_MATRIX FAIL lines above)"
    exit 1
fi

if [ "$WITH_VERB_E2E" = "1" ]; then
    echo "== stage 6: verb e2e (real cerulion binary) =="
    # The repo is expected mounted at /work (the ros2-bench convention).
    (cd /work && cargo build -p cerulion_cli)
    CERULION=/work/target/debug/cerulion
    export CERULION_ROS2_MIGRATE_TOOL="$TOOL"
    require_ws_and_gitdir "$WS" "$WS_ID" "$GITDIR_ID" "the stage-6 verb e2e" || exit 2
    # ws_git sweeps the ambient GIT_* surface
    # for ITS git, but the ENGINE's git would inherit the harness environment
    # verbatim — an ambient GIT_DIR / GIT_WORK_TREE / GIT_INDEX_FILE would
    # point the migration commit at an EXTERNAL repository. Every
    # verb launch therefore goes through git_env_sweep (the same sweep, nothing
    # forced: hooks + config stay the user's — product behavior; the
    # engine supplies its own identity fallback and reads none from this
    # environment). The sweep is EXERCISED, not assumed: a DECOY
    # repository is planted and the redirection trio aimed at it on every
    # launch; after --write the workspace must carry the migration commit
    # under ws_git (HEAD advanced, no modified tracked files — the engine
    # committed HERE) and the decoy must be untouched (HEAD + tree).
    DECOY="$MATRIX_TMP/decoy"
    mkdir "$DECOY"
    decoy_git() {
        git_env_sweep GIT_CONFIG_GLOBAL=/dev/null GIT_CONFIG_SYSTEM=/dev/null \
            git -C "$DECOY" -c core.hooksPath=/dev/null "$@"
    }
    decoy_git init -q
    echo decoy > "$DECOY/d.txt"
    decoy_git add -A
    decoy_git -c user.email=decoy@example.invalid -c user.name=decoy commit -qm decoy
    DECOY_HEAD=$(decoy_git rev-parse HEAD)
    # The engine is launched inside the verified
    # workspace object (ws_bound_launch: cd + verify `.` + the gitdir,
    # then exec) with `--workspace .`, so it starts in the object the
    # check saw rather than at a pathname a same-user writer can swap
    # after the check (the ws_git seam, re-found on the CLI
    # launch — with hooks ON by design, a replacement's pre-commit ran).
    # Residual, stated: the engine canonicalizes `.` to a pathname at
    # startup and addresses the workspace by it for its whole run — an
    # ENGINE property; a swap in that window is DETECTED below (the
    # bound object did not get the commit), never reported as success.
    # The launch sweep keeps the user's
    # config + identity environment (git_env_sweep --keep-user-config's
    # stated keep-list) and strips only the redirection + exec families,
    # so the e2e drives the CLI the way a user's shell would — the
    # hostile decoy trio still rides every launch, and a user-style
    # config + identity is supplied the same way and ASSERTED on the
    # migration commit (env-injected user.email, env author/committer).
    verb() {
        GIT_DIR="$DECOY/.git" GIT_WORK_TREE="$DECOY" GIT_INDEX_FILE="$DECOY/.git/index" \
        GIT_CONFIG_COUNT=1 GIT_CONFIG_KEY_0=user.email GIT_CONFIG_VALUE_0=matrix-verb@example.invalid \
        GIT_AUTHOR_NAME="matrix verb" GIT_COMMITTER_NAME="matrix verb" \
            ws_bound_launch "$CERULION" ros2 migrate --workspace . "$@"
    }
    WS_HEAD_BEFORE=$(ws_git rev-parse HEAD)

    # The verb's outputs are RESULTS — captured create-new into
    # the private results root, sealed, digested, consumed under
    # consume_results like every prover result.
    mkdir -m 0700 "$RESULTS_ROOT/stage6" "$RESULTS_ROOT/stage6b" || exit 2
    ws_capture_result "$RESULTS_ROOT/stage6/dry1.txt" verb || exit 2
    ws_capture_result "$RESULTS_ROOT/stage6/dry2.txt" verb || exit 2
    ws_results_seal "$RESULTS_ROOT/stage6" || exit 2
    STAGE6_SET=$'dry1.txt\ndry2.txt'
    STAGE6_DIGEST=$(ws_results_digest "$RESULTS_ROOT/stage6" "$STAGE6_SET") || exit 2
    consume_results "the stage-6 dry-run determinism compare" "$RESULTS_ROOT/stage6" "$STAGE6_DIGEST" "$STAGE6_SET" \
        cmp "$RESULTS_ROOT/stage6/dry1.txt" "$RESULTS_ROOT/stage6/dry2.txt"
    echo "dry-run determinism OK"
    test -f "$WS/.cerulion/ros2-migrate-manifest.json"
    echo "manifest written OK"

    verb --write --yes
    test -f "$WS/cerulion-ros2-migration.patch"
    ws_git apply -R --check cerulion-ros2-migration.patch
    echo "patch reverses cleanly OK"
    if [ "$(ws_git rev-parse HEAD)" = "$WS_HEAD_BEFORE" ]; then
        echo "STAGE 6 FAIL: the workspace HEAD did not advance — the engine's commit did not land here"
        exit 1
    fi
    if [ -n "$(ws_git status --porcelain --untracked-files=no)" ]; then
        echo "STAGE 6 FAIL: modified tracked files remain after --write — the engine's commit did not land here:"
        ws_git status --porcelain --untracked-files=no
        exit 1
    fi
    if [ "$(decoy_git rev-parse HEAD)" != "$DECOY_HEAD" ] || [ -n "$(decoy_git status --porcelain)" ]; then
        echo "STAGE 6 FAIL: the decoy repository changed — an ambient GIT_* reached the engine's git"
        exit 1
    fi
    echo "migration commit confined to the workspace OK (decoy untouched under hostile GIT_DIR/GIT_WORK_TREE/GIT_INDEX_FILE)"
    IDENT=$(ws_git log -1 --format='%an <%ae>')
    if [ "$IDENT" != "matrix verb <matrix-verb@example.invalid>" ]; then
        echo "STAGE 6 FAIL: the migration commit does not carry the user's environment config + identity (got '$IDENT') — the verb launch stripped the user's config"
        exit 1
    fi
    echo "user config + identity reached the verb OK ($IDENT)"

    echo "== stage 6b: post-migration behavior pin =="
    require_ws_and_gitdir "$WS" "$WS_ID" "$GITDIR_ID" "the stage-6b behavior pin" || exit 2
    (cd "$WS" && RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
        ./build/migrate_fixture_pkg/matrix_runner)

    ws_capture_result "$RESULTS_ROOT/stage6b/after.txt" verb || exit 2
    ws_results_seal "$RESULTS_ROOT/stage6b" || exit 2
    STAGE6B_DIGEST=$(ws_results_digest "$RESULTS_ROOT/stage6b" "after.txt") || exit 2
    no_diff_proposed() {
        ! grep -q "^--- a/" "$RESULTS_ROOT/stage6b/after.txt"
    }
    if consume_results "the stage-6b idempotence check" "$RESULTS_ROOT/stage6b" "$STAGE6B_DIGEST" "after.txt" \
            no_diff_proposed; then
        echo "idempotence OK"
    else
        echo "IDEMPOTENCE FAIL: second dry-run still proposes a diff"
        exit 1
    fi
fi

echo "run_matrix: ALL STAGES OK"
