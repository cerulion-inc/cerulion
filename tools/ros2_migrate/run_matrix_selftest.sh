#!/usr/bin/env bash
# Self-test for matrix_lib.sh (the hostile-workspace execution
# defenses). DESK-RUNNABLE: bash + git only — no ROS, no container — so
# the defenses are provable on the git that will actually run them.
#
#   1. hook canary: a planted pre-commit / post-checkout hook does NOT
#      execute through ws_git — and the CONTROL proves the SAME hook
#      DOES fire through plain (ambient-config-shielded) git, so the
#      canary cannot pass vacuously and this git provably honors the
#      core.hooksPath override.
#   2. identity gate: require_ws_identity passes on the untouched bound
#      object and REFUSES after the workspace pathname is swapped.
#   3. smudge-filter canary (config-FILE channel): a hostile GLOBAL
#      config supplying filter.evil.smudge does not execute through
#      ws_git; the control fires it through plain git.
#   4. gitdir gate THROUGH ws_git: the gate is folded into
#      the wrapper, so a swapped .git makes ws_git itself refuse — no
#      caller discipline required. The standalone helper for the
#      non-git stages is pinned in the same arm.
#   5. runtime config-injection canary (env channel): a
#      GIT_CONFIG_COUNT/KEY_n/VALUE_n-injected filter does not execute
#      through ws_git (the wrapper clears the ambient GIT_* surface);
#      the control fires it through plain git.
#   6. ambient bind variables are DISOWNED: the production
#      publish -> init -> gitdir-bind -> add -> commit sequence, run in a
#      child shell whose ENVIRONMENT carries bogus inherited WS_BIND_*
#      (the shape that made the matrix exit 2 before setup), proceeds
#      normally and names what it ignored; plus the structural pin that
#      run_matrix.sh publishes ONLY through the disowning seams.
#   7. workspace OBJECT binding: a ws_git parked at its
#      bind/verify -> exec instant has the whole workspace pathname
#      swapped for a VALID hostile repository (local-config smudge
#      filter, different content); git must operate on the ORIGINAL
#      object or refuse — never the swap. The CONTROL runs pathname git
#      after the same swap and lands in the hostile tree, filter fired.
#   8. gitdir OBJECT binding (fd-bound — Linux only): the
#      same park, then only `.git` is swapped INSIDE the bound
#      workspace; the fd-bound gitdir keeps git on the original object
#      (original content, no filter) while the by-name control reads
#      the swap and fires. Skipped LOUDLY on hosts without /dev/fd
#      traversal (macOS), where run_matrix.sh refuses to run at all.
#   9. git_env_sweep confinement: a plain git started
#      INSIDE a workspace the way the engine starts its own (`git -C`)
#      under hostile inherited GIT_DIR / GIT_WORK_TREE / GIT_INDEX_FILE
#      aimed at an EXTERNAL repository commits in the workspace, not the
#      external repo; the CONTROL without the sweep lands in the
#      external repo. Plus the structural pin that every launch of the
#      real verb in run_matrix.sh goes through the sweep.
#  10. inherited fd-binding verdict: a WS_GIT_FD_BINDING
#      inherited from the environment — bare or pid-stamped by another
#      process, and INVERTED against this host's real verdict — never
#      overrides the probe. The host's verdict is an
#      INDEPENDENT oracle (git --git-dir=/dev/fd/N + stat -L on a
#      throwaway repo, computed at the arm-8 gate), which the harness
#      function must agree with and the children are compared against.
#  11. keep-config sweep: git_env_sweep --keep-user-config
#      still confines git under the hostile redirection trio AND passes
#      the user's config + identity through (asserted on the commit's
#      author/email/committer and as the EXACT GIT_* set the launched
#      command sees); the default sweep passes nothing.
#  12. bound launch: ws_bound_launch parked at
#      its bind/verify -> exec instant has the workspace pathname swapped
#      for a hostile repo carrying a pre-commit CANARY; a stand-in for
#      the engine (hooks ON, a hostile GIT_DIR in its environment) must
#      run in the ORIGINAL object, the canary silent, the external repo
#      untouched; the pathname CONTROL fires the canary. Plus the pins
#      that stage 6 launches through ws_bound_launch with `--workspace .`.
#  13. FIFO at `.git`: a writerless FIFO replacing `.git`
#      makes ws_bind_gitdir AND ws_git REFUSE within a bounded wall (a
#      `timeout`-wrapped call must exit nonzero and not be the one killed)
#      — the type gate runs before any open, so nothing reads or waits.
#  14. symlinked `.git`: a `.git` that is a symlink to a real
#      gitdir is refused at the bind and at every ws_git — never followed
#      into a redirected gitdir; plus the structural pin that the bind
#      derives WS_BIND_GITDIR_ID from the opened DESCRIPTOR on fd hosts.
#  15. analysis-input descendants: ws_bind_descendant binds a
#      descendant directory from inside the bound workspace, refuses a
#      symlinked one, and require_ws_descendant refuses after a swap;
#      plus the pin that run_matrix.sh binds `src` + the compile-db dir
#      and invokes the prover ONLY through `analyze`.
#  16. symlink-to-same-inode: a bound directory moved aside
#      and its pathname replaced by a symlink to that very inode carries
#      the bound identity — the pathname checks (descendant, workspace,
#      gitdir) must refuse it before any identity comparison.
#  17. content bind: a tree digest changes when one file's
#      bytes change under an unchanged directory identity, and the
#      digest check refuses; plus the pin that run_matrix.sh captures
#      the source + compile-db digests before analysis and re-verifies
#      them after stages 3, 3b, 4 and 5.
#  18. symlinked INTERMEDIATE component: the bound tree moved
#      aside and `build` linked to it — the final component still carries
#      the bound identity — is refused by the bind and by the recheck
#      (the real-directory control passes).
#  19. byte-exact link targets: two symlinks whose targets
#      differ only by a trailing newline digest DIFFERENTLY; identical
#      trees digest identically.
#  20. FIFO in place of the compile database: the capture and
#      the recheck refuse inside a timeout bound, never hang.
#  21. immutable verified copy: a
#      stand-in prover, parked by a handshake, has the LIVE source swapped
#      for a marker file mid-run and restored afterwards; the analysis
#      output must carry the APPROVED bytes, never the marker, and the
#      snapshot must verify against the approved digest; plus the pins
#      that run_matrix.sh snapshots the inputs and runs the prover only
#      through ws_analyze_snapshot.
#  22. the snapshot is SEALED read-only: files AND directories
#      lose their write bits and the root goes 0500 before the first
#      prover run, and every stage re-checks the seal and re-digests the
#      SNAPSHOT itself. Prevention for an unprivileged writer; detection
#      for a same-UID one (a swap timed against the snapshot).
#  23. prover results are sealed too:
#      every result is captured create-new into a fresh private results
#      root, sealed at the producer's exit, digested over the EXACT
#      expected file set, and every consumer re-verifies that digest
#      immediately before and after reading; a result forged between
#      the prover's exit and the consumer is refused with nothing applied.
#  24. the read watchdog is a BOUND, not a per-file toll: every
#      watchdog subshell redirects its stdout, so the `sleep` child that
#      outlives the reaped subshell cannot hold a command substitution's
#      pipe open for the whole timeout; five digests must cost less than
#      ONE bound (the defect cost five), the FIFO bound must still fire,
#      and the digest itself must be unchanged.
#  25. the result CAPTURE is create-new AND bounded: `noclobber`
#      reads like a create-new guard but is not a bound — an existing
#      writerless FIFO at the destination BLOCKS the redirect — so the
#      capture refuses any destination that already exists and carries the
#      same watchdog every other seam does — around the OPEN only, never
#      around the prover, which legitimately runs for many seconds.
#  26. a failed file read REFUSES the tree digest, never truncates it: a
#      read that fails must reach the caller instead of yielding a
#      shorter digest with rc 0 — asserted on every uid through a PATH
#      read-failure seam, and again through a real OS denial when not root.
#  27. mutant prover binaries never outlive the matrix:
#      an aborted mutant run — by a plain failure exit AND by a
#      signal, which bash DOES run the EXIT trap for (the measurement
#      sub-arm (c) stands on) — must leave no
#      mutant binary in the tool directory and must leave the production
#      prover byte-identical to the one the run built; a mutant binary
#      already on disk is REFUSED rather than deleted; plus the structural
#      pins that every mutant run_matrix.sh builds is registered and that
#      its EXIT trap — the single trap it installs — restores them. In
#      addition, the model child BINDS its mutant's identity after "building" it,
#      as production does, and a registered entry with NO identity (a
#      dangling link, a build that never completed) is refused and named
#      by the sweep, never swept blind (sub-arm (h4)).
#  28. the prover's total-switch guard is ARMED:
#      `-Werror=switch` sits AFTER `$(llvm-config --cxxflags)` so an
#      ordinary `-Wno-switch` is overridden, AND the build REFUSES any
#      warning-suppressing flag out of any llvm-config expansion — because
#      `-w` suppresses in either position and --ldflags/--libs/--system-libs
#      land after the promotion, so no ordering could cover them. Driven
#      through the real build.sh with a stub toolchain that records argv.
#      Sub-arm (e) covers `--out-dir DIR`: the binary lands at
#      DIR/<the same name>, in either flag order; a missing, symlinked or
#      repeated DIR, a valueless flag and an unknown flag are refused
#      before the compiler runs; and the plain build still lands at the
#      shared name.
#  29. the pristine prover is copied THROUGH its descriptor:
#      a FIFO or symlink at the prover path is refused
#      instantly by the caller's type gate (never after the bound — a TERM
#      inside a builtin runs no EXIT trap); the helper's symlink gate,
#      descriptor test, watchdog and copy-failure branch are each refused by
#      a DIFFERENT fixture; the watchdog REFUSES rather than killing its
#      caller; and a pathname swapped mid-copy does not change the bytes.
#  30. the continuation join is bash-faithful:
#      an escaped (even) backslash run ends the line, a spliced comment ends
#      the command, a comment never swallows the line below it, a TRAILING
#      comment is not code, a genuine continuation is still joined, and a
#      site mentioned in a comment is not a site — the shapes that decide
#      whether a source walk can MISS a build site.
#  31. the mutant registry binds the ARTIFACT, not the pathname:
#      a registered name whose object was replaced —
#      by a different file, by a byte-identical copy under a new inode, or
#      by an in-place rewrite of the same size — is REFUSED by the
#      per-mutant removal AND by the EXIT-trap sweep (reported, latched as a
#      restore failure, never deleted); a path this run never registered is
#      refused; an entry whose build never completed — NO identity — is
#      refused, named and latched by both removal paths, never swept blind
#      (no identity ⇒ no removal; nothing at such a name is not a
#      failure); a removed name is UNREGISTERED so a later occupant is
#      reported as what it is; the pre-run verify passes only the bound
#      artifact; a SYMLINK at a bound name is refused everywhere (the key
#      never follows) and the bind refuses a link or an unregistered path; a
#      name's second life begins unbound (re-registering clears the first
#      life's identity, so an interrupted rebuild is refused as a build that
#      never completed, never as a swap); a malformed identity — no
#      sub-second fraction, or not one line of digits and colons — is
#      refused fail-closed at the CONSUMERS (the bind's identity
#      comes from the stamped descriptor, so the stub stat drives the
#      verify and the removals) and reported as "cannot be identified",
#      never as a swap (a stub rendering the bound identity passes — the
#      control); a bound artifact that
#      VANISHED is said out loud by both removal paths; a removal `rm`
#      refuses (a stub rm, for every uid) carries rm's reason and latches,
#      and a directory at a never-bound name is refused before any rm;
#      the bound identity carries a RUN-OWNED mtime stamp, so a
#      same-inode, same-size rewrite that a coalesced clock lands on the
#      build's own tick (modelled by resetting the mtime) is still refused,
#      a write landing after the stamp is caught by name at the next check;
#      the stamp and the identity go through ONE O_NOFOLLOW
#      descriptor (a link swapped in after the by-name gate is refused by
#      the kernel and its target untouched), the descriptor's identity
#      equals the name's stat rendering (parity), one run per tool
#      directory is held by a kernel lock on the DIRECTORY's own descriptor
#      verified after the open (no lock file; a link swapped in after the
#      by-name gate is refused; a second run refused by name; released on
#      exit), every mutant is
#      built into a run-PRIVATE root that is minted after the bind, swept
#      with the run and refused at the next startup if left behind; and
#      run_matrix.sh locks before the leftover check, mints the root after
#      the prover bind, builds into it, binds after the build, verifies
#      before the run, removes NOTHING after a failed bind, and removes
#      through the seam at every site — no `rm` of any spelling on a
#      mutant path.
#
# Every ws_git call after each arm's bind mirrors production: the bind
# variables are published through the same seams (ws_bind_publish /
# ws_bind_gitdir) before git runs, so the wrapper's own gate is
# exercised on every arm, not just arm 4.
set -euo pipefail
cd "$(dirname "$0")"
# shellcheck source=matrix_lib.sh
. ./matrix_lib.sh

T=$(mktemp -d)
trap 'chmod -R u+w "$T" 2>/dev/null; rm -rf "$T"' EXIT # a sealed snapshot left by a failing arm must still go
fail() {
    echo "SELFTEST FAIL: $1" >&2
    exit 1
}

# Join shell line continuations the way bash does, and drop comments — the
# view the two walks that need it match against: arm 23's consumer and
# forbidden-path greps, and arm 27's build-site walk. Every other grep below
# matches the raw file or `$CODE_FILE` deliberately. Those two walks match
# within ONE line, so an invocation split across a continuation carries its
# tokens on no single physical line and is INVISIBLE to them.
#
# The join is faithful on the
# three points that decide whether a walk can MISS a site. All three were
# MEASURED against bash, and arm 30 pins each one:
#   * a trailing backslash continues only when the run of backslashes is
#     ODD. An EVEN run is an escaped backslash and ENDS the line. Joining on
#     mere PRESENCE fuses the next line onto this one, and two build sites
#     fused onto one line are counted ONCE — a walk that reports 1 site
#     against 1 registration and passes while a site goes unregistered.
#   * a full-line comment never begins or continues a command: bash ends the
#     comment at its newline whether or not it trails a backslash. So a
#     comment is dropped WITHOUT consuming the next line. Both orderings get
#     this wrong in their own way: joining BEFORE comments are stripped
#     swallows a real command that followed a comment line ending in a
#     backslash (fixture j30c), and stripping comments BEFORE joining fuses
#     the commands on either side of a spliced comment (fixture j30b). Only
#     a join that handles comments ITSELF gets both right.
#   * a comment line spliced INTO a continuation ends the command at its
#     `#`. The following line is a NEW command, and the comment's own
#     trailing backslash does not continue either.
#   * a TRAILING comment ends the command too, so a backslash INSIDE it is
#     not a continuation (fixture j30g). The continuation test therefore
#     runs on the line's CODE part, and the emitted line carries the same
#     `[ \t]#` strip `$CODE` uses — quote-naive, by that same convention.
#
# Stripping comments FIRST and joining on presence is the tempting shape. The
# argument for it is that both divergences are safe because they "push toward
# MORE joining, never less", and that the only wrong fusion — a registration
# and a build landing on one line — fails the ordering check. That argument
# is false in general: fusing TWO BUILD SITES is
# also "more joining", and it hides one.
#
# The next line's leading whitespace is kept, exactly as bash leaves it, so
# tokens stay separated.
join_logical_lines() {
    # $1 file. Echoes one logical line per BACKSLASH-CONTINUED command, with
    # comments dropped. Within a line it tracks escapes and quotes (see
    # code_part); ACROSS lines it does not, so a `#` line inside
    # run_matrix.sh'"'"'s `<<'"'"'PYEOF'"'"'` heredoc body is still dropped as a comment and a
    # `\`-terminated heredoc line is still joined. Other continuation forms
    # (a trailing `&&`, `||`, `|`, a quoted newline) stay split. None of those
    # can hide a build site from the walks below — they overcount or drop a
    # non-site — which is the only direction that matters here. The two shapes
    # that DID hide one, both now pinned, were an escaped space before a `#`
    # and a `#` inside quotes.
    awk '
        function code_part(s,   i, c, p, sq, dq, esc, q1) {
            # A `#` starts a comment only at the START OF A WORD and only
            # outside quotes. A trailing-regex strip gets two shapes wrong in
            # the UNSAFE direction (measured against bash): `cmd \ # note`
            # ends in an ESCAPED SPACE, so stripping the comment leaves a lone
            # trailing backslash and the join then FUSES the next command in —
            # two build sites counted as one, the exact undercount this walk
            # exists to prevent; and a `#` inside quotes (`msg="a # b"`) is not
            # a comment at all. Left-to-right, tracking escapes and quotes.
            # The apostrophe is built from its code point: this awk program
            # lives inside a SINGLE-QUOTED shell string, so a literal one
            # cannot appear here at all.
            q1 = sprintf("%c", 39)
            sq = 0; dq = 0; esc = -1
            for (i = 1; i <= length(s); i++) {
                c = substr(s, i, 1)
                if (c == "\\" && !sq) { esc = i + 1; i++; continue }
                if (c == q1 && !dq) { sq = !sq; continue }
                if (c == "\"" && !sq) { dq = !dq; continue }
                if (c == "#" && !sq && !dq && i > 1) {
                    p = substr(s, i - 1, 1)
                    if ((p == " " || p == "\t") && (i - 1) != esc) {
                        return substr(s, 1, i - 2)
                    }
                }
            }
            return s
        }
        function bs_run(s) { if (match(s, /\\+$/)) return RLENGTH; return 0 }
        {
            line = $0
            sub(/\r$/, "", line)
            if (line ~ /^[ \t]*#/) next
            line = code_part(line)
            while (bs_run(line) % 2 == 1) {
                line = substr(line, 1, length(line) - 1)
                if ((getline nxt) <= 0) break
                sub(/\r$/, "", nxt)
                if (nxt ~ /^[ \t]*#/) break
                line = line code_part(nxt)
            }
            print line
        }' "$1"
}

# A VALID repository standing in for the attacker's replacement: a smudge
# filter in its LOCAL config (the channel the identity gate exists to
# close), named from BOTH the worktree's .gitattributes and the gitdir's
# info/attributes (so a swap of EITHER half carries the attribute),
# f.txt committed as 'hostile' and then REMOVED from the worktree so a
# checkout landing there materializes it. Ungated git succeeds here —
# which is what makes the arms' "never the swapped tree" oracles
# non-vacuous (an invalid replacement hands the arm
# git's own error and a deleted gate still passes).
make_hostile_repo() {
    # $1 dir, $2 sentinel path the smudge writes.
    mkdir "$1"
    (
        cd "$1"
        export GIT_CONFIG_GLOBAL=/dev/null GIT_CONFIG_SYSTEM=/dev/null
        git init -q
        echo hostile > f.txt
        printf 'f.txt filter=evil\n' > .gitattributes
        printf 'f.txt filter=evil\n' > .git/info/attributes
        git config filter.evil.smudge "sh -c 'echo pwned > \"$2\"; cat'"
        git -c user.email=evil@example.invalid -c user.name=evil add -A
        git -c user.email=evil@example.invalid -c user.name=evil commit -qm hostile
        rm f.txt
    )
}

# Park/release handshake for a background bound wrapper (its
# CERULION_WS_PAUSE_DIR seam): wait_paused returns once the wrapper
# has bound + verified and is parked before its exec — bounded, so a
# wrapper that never reaches the seam FAILS the arm instead of hanging
# it (the ready FIFO is opened read-write here, which never blocks);
# release_paused lets it exec git (a blocking write-open: the wrapper is
# already reading, so the line cannot be discarded with the pipe).
pause_dir_new() {
    local d="$T/pause$1"
    mkdir "$d" && mkfifo "$d/ready" "$d/go"
    printf '%s' "$d"
}
wait_paused() {
    # $1 pause dir, $2 optional label for the failure (default: background ws_git).
    local rfd=
    exec {rfd}<>"$1/ready"
    read -r -t 60 -u "$rfd" _ \
        || fail "${2:-background ws_git} never reached its pause seam"
    exec {rfd}<&-
}
release_paused() {
    echo go > "$1/go"
}
# Ambient-config-shielded plain git for repositories the selftest itself
# owns (controls, decoys) — never the wrapper under test.
plain_git() {
    GIT_CONFIG_GLOBAL=/dev/null GIT_CONFIG_SYSTEM=/dev/null \
        git -c core.hooksPath=/dev/null "$@"
}
# The INDEPENDENT fd-binding oracle — not the harness
# function under test, but the capability the harness relies on,
# exercised directly on a throwaway repo: git must accept an fd-bound
# gitdir (`--git-dir=/dev/fd/N`) AND the opened descriptor must stat to
# the pathname's own dev:inode (`stat -L`). Both hold on Linux procfs;
# neither on macOS devfs (measured).
independent_fd_verdict() {
    local d fd='' v=0
    d=$(mktemp -d "$T/fdprobe.XXXXXX")
    (cd "$d" && plain_git init -q)
    { exec {fd}<"$d/.git"; } 2>/dev/null || true
    if [ -n "$fd" ] \
        && [ "$(stat -L -c %d:%i "/dev/fd/$fd" 2>/dev/null || stat -L -f %d:%i "/dev/fd/$fd" 2>/dev/null)" \
            = "$(stat -c %d:%i "$d/.git" 2>/dev/null || stat -f %d:%i "$d/.git")" ] \
        && (cd "$d" && plain_git --git-dir="/dev/fd/$fd" rev-parse --git-dir >/dev/null 2>&1); then
        v=1
    fi
    if [ -n "$fd" ]; then
        exec {fd}<&-
    fi
    rm -rf "$d"
    echo "$v"
}

# The host's fd-binding verdict comes from the INDEPENDENT
# probe, and the harness function must agree with it BEFORE any arm
# rides its verdict (an inverted probe would otherwise be caught later,
# by ws_git's own fd-identity gate rather than by this oracle).
HOST_FD=$(independent_fd_verdict)
if ws_git_fd_binding_available; then FN_FD=1; else FN_FD=0; fi
[ "$FN_FD" = "$HOST_FD" ] \
    || fail "ws_git_fd_binding_available reports $FN_FD but this host's fd binding is really $HOST_FD (independent probe: git --git-dir=/dev/fd/N + stat -L)"
echo "fd-binding oracle: independent verdict $HOST_FD; the harness function agrees"

echo "== selftest 1: hook canary =="
unset WS_BIND_PATH WS_BIND_ID WS_BIND_GITDIR_ID
WS1="$T/ws1"
mkdir "$WS1"
ws_bind_publish "$WS1" "$(ws_identity "$WS1")"
echo hello > "$WS1/f.txt"
ws_git init -q
ws_bind_gitdir
mkdir -p "$WS1/.git/hooks"
cat > "$WS1/.git/hooks/pre-commit" <<EOH
#!/bin/sh
echo pwned > "$T/sentinel-commit"
EOH
cat > "$WS1/.git/hooks/post-checkout" <<EOH
#!/bin/sh
echo pwned > "$T/sentinel-checkout"
EOH
chmod +x "$WS1/.git/hooks/pre-commit" "$WS1/.git/hooks/post-checkout"
ws_git add -A
ws_git -c user.email=matrix@example.invalid -c user.name=matrix \
    commit -qm canary
[ ! -e "$T/sentinel-commit" ] || fail "pre-commit hook EXECUTED through ws_git"
echo changed > "$WS1/f.txt"
ws_git checkout -- f.txt
[ ! -e "$T/sentinel-checkout" ] || fail "post-checkout hook EXECUTED through ws_git"
# CONTROL (anti-tautology): the same planted hook DOES fire through git
# WITHOUT the wrapper's hooksPath override — the canary apparatus works,
# and the suppression above is therefore the wrapper's doing. Ambient
# global/system config is still shielded so a desk-level hooksPath
# (husky et al.) cannot fake a pass or a failure.
echo more >> "$WS1/f.txt"
GIT_CONFIG_GLOBAL=/dev/null GIT_CONFIG_SYSTEM=/dev/null \
    git -C "$WS1" add -A
GIT_CONFIG_GLOBAL=/dev/null GIT_CONFIG_SYSTEM=/dev/null \
    git -C "$WS1" -c user.email=matrix@example.invalid -c user.name=matrix \
    commit -qm control
[ -e "$T/sentinel-commit" ] || fail "control: plain git did not run the planted hook (apparatus broken)"
echo "hook canary OK (ws_git suppressed pre-commit + post-checkout; control fired)"

echo "== selftest 2: identity gate =="
WS2="$T/ws2"
mkdir "$WS2"
ID=$(ws_identity "$WS2")
require_ws_identity "$WS2" "$ID" "the selftest happy arm" \
    || fail "identity gate refused the untouched bound object"
mv "$WS2" "$T/aside"
mkdir "$WS2" # a swapped-in replacement at the same pathname
if require_ws_identity "$WS2" "$ID" "the selftest swap arm" 2>/dev/null; then
    fail "identity gate ACCEPTED a swapped workspace"
fi
echo "identity gate OK (untouched passes; swap refused)"

echo "== selftest 3: smudge-filter canary (config-file channel) =="
# Attributes only NAME a filter — the command comes from
# config. A hostile GLOBAL config supplying filter.evil.smudge models
# the config channel; ws_git nulls that layer, so the checkout must
# pass the content through and execute nothing. The CONTROL runs the
# same checkout with the hostile global honored, proving the channel is
# real and the suppression above is ws_git's doing. (The LOCAL-config
# channel — a replaced .git — is closed by the gitdir gate, arm 4.)
unset WS_BIND_PATH WS_BIND_ID WS_BIND_GITDIR_ID
WS3="$T/ws3"
mkdir "$WS3"
ws_bind_publish "$WS3" "$(ws_identity "$WS3")"
echo hello > "$WS3/f.txt"
printf 'f.txt filter=evil\n' > "$WS3/.gitattributes"
ws_git init -q
ws_bind_gitdir
ws_git add -A
ws_git -c user.email=matrix@example.invalid -c user.name=matrix \
    commit -qm baseline
HOSTILE_CFG="$T/hostile-gitconfig"
git config -f "$HOSTILE_CFG" filter.evil.smudge \
    "sh -c 'echo pwned > \"$T/sentinel-smudge\"; cat'"
rm "$WS3/f.txt"
GIT_CONFIG_GLOBAL="$HOSTILE_CFG" ws_git checkout -- f.txt
[ ! -e "$T/sentinel-smudge" ] || fail "smudge filter EXECUTED through ws_git"
[ "$(cat "$WS3/f.txt")" = "hello" ] \
    || fail "passthrough did not restore the content"
# CONTROL (anti-tautology): the same hostile global honored -> the
# filter fires on the same checkout.
rm "$WS3/f.txt"
GIT_CONFIG_GLOBAL="$HOSTILE_CFG" GIT_CONFIG_SYSTEM=/dev/null \
    git -C "$WS3" checkout -- f.txt
[ -e "$T/sentinel-smudge" ] \
    || fail "control: the hostile global filter did not fire (apparatus broken)"
echo "smudge canary OK (ws_git passed through; control fired)"

echo "== selftest 4: gitdir gate through ws_git =="
unset WS_BIND_PATH WS_BIND_ID WS_BIND_GITDIR_ID
WS4="$T/ws4"
mkdir "$WS4"
ws_bind_publish "$WS4" "$(ws_identity "$WS4")"
ws_git init -q
ws_bind_gitdir
ws_git status >/dev/null \
    || fail "gated ws_git refused the untouched workspace + gitdir"
mv "$WS4/.git" "$T/aside-git"
# The replacement must be a VALID gitdir — an empty dir would make git
# itself fail "not a repository", handing the arm a nonzero exit even
# with the gate deleted (an empty directory would leave the oracle
# vacuous). With a real repo swapped in, ungated git would succeed, so
# only the gate can produce the refusal — and the arm additionally
# discriminates ON the identity message, so no other error can stand in.
EVIL="$T/evil-repo"
mkdir "$EVIL"
GIT_CONFIG_GLOBAL=/dev/null GIT_CONFIG_SYSTEM=/dev/null git -C "$EVIL" init -q
mv "$EVIL/.git" "$WS4/.git" # a swapped-in VALID replacement gitdir
if OUT=$(ws_git status 2>&1); then
    fail "ws_git EXECUTED against a swapped gitdir"
fi
case "$OUT" in
    *"no longer carries the bound identity"*) ;;
    *) fail "ws_git refused for the wrong reason (not the identity gate): $OUT" ;;
esac
# The standalone helper (still guarding the non-git stages) refuses too.
if require_ws_and_gitdir "$WS4" "$WS_BIND_ID" "$WS_BIND_GITDIR_ID" \
    "the selftest standalone arm" 2>/dev/null; then
    fail "standalone gate ACCEPTED a swapped gitdir"
fi
echo "gitdir gate OK (ws_git self-refuses; standalone helper refuses)"

echo "== selftest 5: runtime config-injection canary (env channel) =="
# GIT_CONFIG_COUNT + KEY_n/VALUE_n inject config at
# command-line precedence — above every file the wrapper nulls. The
# wrapper clears the whole ambient GIT_* surface, so the injected
# filter must not execute; the control fires it through plain git.
unset WS_BIND_PATH WS_BIND_ID WS_BIND_GITDIR_ID
ws_bind_publish "$WS3" "$(ws_identity "$WS3")"
ws_bind_gitdir
rm "$WS3/f.txt"
GIT_CONFIG_COUNT=1 \
    GIT_CONFIG_KEY_0=filter.evil.smudge \
    GIT_CONFIG_VALUE_0="sh -c 'echo pwned > \"$T/sentinel-envcfg\"; cat'" \
    ws_git checkout -- f.txt
[ ! -e "$T/sentinel-envcfg" ] \
    || fail "env-injected smudge filter EXECUTED through ws_git"
[ "$(cat "$WS3/f.txt")" = "hello" ] \
    || fail "passthrough did not restore the content (env arm)"
# CONTROL (anti-tautology): the same injection fires through plain git
# (file configs shielded so only the env channel is under test).
rm "$WS3/f.txt"
GIT_CONFIG_COUNT=1 \
    GIT_CONFIG_KEY_0=filter.evil.smudge \
    GIT_CONFIG_VALUE_0="sh -c 'echo pwned > \"$T/sentinel-envcfg\"; cat'" \
    GIT_CONFIG_GLOBAL=/dev/null GIT_CONFIG_SYSTEM=/dev/null \
    git -C "$WS3" checkout -- f.txt
[ -e "$T/sentinel-envcfg" ] \
    || fail "control: the env-injected filter did not fire (apparatus broken)"
echo "env-injection canary OK (ws_git cleared it; control fired)"

echo "== selftest 6: ambient bind variables are disowned =="
# The child shell's ENVIRONMENT carries bogus WS_BIND_* — exactly what a
# caller who exported them (a debugging shell, a wrapper script) hands
# run_matrix.sh. Before the disown fix the inherited WS_BIND_GITDIR_ID reached
# the harness's own `ws_git init` before any gitdir existed and the
# wrapper refused (exit 2 before setup). The production sequence must
# now proceed normally AND say what it ignored.
WS6="$T/ws6"
mkdir "$WS6"
echo hello > "$WS6/f.txt"
if ! OUT6=$(WS_BIND_PATH=/nonexistent WS_BIND_ID=1:1 WS_BIND_GITDIR_ID=99999:99999 \
    bash -c '
        set -euo pipefail
        . ./matrix_lib.sh
        ws_bind_publish "$1" "$(ws_identity "$1")"
        ws_git init -q
        ws_bind_gitdir
        ws_git add -A
        ws_git -c user.email=matrix@example.invalid -c user.name=matrix \
            commit -qm baseline
        ws_git status --porcelain
    ' _ "$WS6" 2>&1); then
    fail "the production setup sequence REFUSED under inherited WS_BIND_*: $OUT6"
fi
# EVERY inherited variable must be named — a disown that
# handled only the gitdir id while leaving PATH/ID exported would still
# pass the sequence.
for bind in WS_BIND_PATH WS_BIND_ID WS_BIND_GITDIR_ID; do
    case "$OUT6" in
        *"ignoring ambient $bind="*) ;;
        *) fail "the ambient $bind was not named as ignored: $OUT6" ;;
    esac
done
[ -d "$WS6/.git" ] || fail "the sequence did not init the workspace"
if printf '%s\n' "$OUT6" | grep -v '^warning: ' | grep -q .; then
    fail "status --porcelain after the sequence is not clean: $OUT6"
fi
# The harness must publish ONLY through the disowning seams: no direct
# WS_BIND_* assignment anywhere in run_matrix.sh (a commented-out one
# fails this too — loudly, never vacuously), both seams and the fd
# requirement called from CODE (comment-stripped view), and no -C
# handed to ws_git (the wrapper refuses it).
if grep -nE '^[[:space:]]*(export[[:space:]]+)?WS_BIND_(PATH|ID|GITDIR_ID)=' run_matrix.sh; then
    fail "run_matrix.sh assigns a WS_BIND_* directly — publish through ws_bind_publish / ws_bind_gitdir"
fi
CODE=$(sed -e 's/[[:space:]]#.*$//' -e '/^[[:space:]]*#/d' run_matrix.sh)
# The stripped view also lives in a FILE: `printf "$CODE" | grep -q` under
# pipefail fails on TIMING (grep -q exits at the first match, printf's
# remaining write takes EPIPE) — every -q pin greps the file.
CODE_FILE="$T/run_matrix.code"
printf '%s\n' "$CODE" > "$CODE_FILE"
grep -qE '^[[:space:]]*ws_bind_publish[[:space:]]' "$CODE_FILE" \
    || fail "run_matrix.sh does not publish its workspace binding through ws_bind_publish"
grep -qE '^[[:space:]]*ws_bind_gitdir([[:space:]]|$)' "$CODE_FILE" \
    || fail "run_matrix.sh does not bind its gitdir through ws_bind_gitdir"
grep -qE '^[[:space:]]*ws_git_require_fd_binding' "$CODE_FILE" \
    || fail "run_matrix.sh does not require fd binding up front"
if printf '%s\n' "$CODE" | grep -nE 'ws_git[[:space:]]+-C[[:space:]]'; then
    fail "run_matrix.sh passes -C to ws_git — the wrapper refuses it (a pathname there re-targets git past the binding)"
fi
# ORDER: the disowning publish must precede the harness's FIRST ws_git
# — that is what makes "disown at the ownership seam" equivalent to
# "disown at startup" (nothing between them consults WS_BIND_*, and
# ws_git refuses to run without a published binding).
PUB_LINE=$(printf '%s\n' "$CODE" | grep -nE '^[[:space:]]*ws_bind_publish[[:space:]]' | head -1 | cut -d: -f1)
GIT_LINE=$(printf '%s\n' "$CODE" | grep -nE '(^|[^_a-zA-Z0-9])ws_git[[:space:]]' | head -1 | cut -d: -f1)
[ -n "$GIT_LINE" ] || fail "run_matrix.sh never calls ws_git (pin apparatus broken)"
[ "$PUB_LINE" -lt "$GIT_LINE" ] \
    || fail "run_matrix.sh calls ws_git (code line $GIT_LINE) before it publishes its binding (code line $PUB_LINE)"
echo "ambient disown OK (inherited WS_BIND_* ignored + named; harness publishes only through the seams)"

echo "== selftest 7: git is bound to the verified workspace OBJECT =="
unset WS_BIND_PATH WS_BIND_ID WS_BIND_GITDIR_ID
WS7="$T/ws7"
mkdir "$WS7"
ws_bind_publish "$WS7" "$(ws_identity "$WS7")"
echo original > "$WS7/f.txt"
ws_git init -q
ws_bind_gitdir
ws_git add -A
ws_git -c user.email=matrix@example.invalid -c user.name=matrix \
    commit -qm original
make_hostile_repo "$T/evil7" "$T/sentinel-swap7"
rm "$WS7/f.txt"
P7=$(pause_dir_new 7)
CERULION_WS_PAUSE_DIR="$P7" ws_git checkout -- f.txt > "$T/out7" 2>&1 &
PID7=$!
wait_paused "$P7" # bound + verified, parked before the exec
mv "$WS7" "$T/aside7"
mv "$T/evil7" "$WS7" # the reproduced shape: a valid hostile repo at the checked pathname
release_paused "$P7"
if wait "$PID7"; then RC7=0; else RC7=$?; fi
# The ONLY acceptable outcomes: git operated on the ORIGINAL object, or
# the wrapper refused. Never the swapped tree — in either form.
[ ! -e "$T/sentinel-swap7" ] \
    || fail "ws_git EXECUTED the swapped-in tree's smudge filter (check->exec seam open)"
[ ! -e "$WS7/f.txt" ] \
    || fail "ws_git materialized f.txt INTO the swapped-in tree"
if [ "$RC7" -eq 0 ]; then
    [ "$(cat "$T/aside7/f.txt")" = original ] \
        || fail "ws_git succeeded but did not restore the ORIGINAL object's f.txt"
    echo "swap after bind: git operated on the original object (restored aside7/f.txt; hostile tree untouched)"
else
    case "$(cat "$T/out7")" in
        *"no longer carries the bound identity"*) echo "swap after bind: ws_git refused (identity gate)" ;;
        *) fail "ws_git failed for a reason other than the identity gate: $(cat "$T/out7")" ;;
    esac
fi
# CONTROL (anti-tautology): the SAME swap, then git started BY PATHNAME
# with only the hostile LOCAL config in play — lands in the hostile tree
# and fires its filter, so the apparatus is real and the outcome above
# is the binding's doing.
GIT_CONFIG_GLOBAL=/dev/null GIT_CONFIG_SYSTEM=/dev/null \
    git -C "$WS7" checkout -- f.txt
[ -e "$T/sentinel-swap7" ] \
    || fail "control: pathname git did not run the hostile local filter (apparatus broken)"
[ "$(cat "$WS7/f.txt")" = hostile ] \
    || fail "control: pathname git did not materialize the hostile tree's file (apparatus broken)"
echo "workspace object binding OK (ws_git stayed on the original; pathname control landed in the swap and fired)"

echo "== selftest 8: git is bound to the verified GITDIR object (fd-bound) =="
# Gated on the INDEPENDENT oracle (validated against the
# harness function in the preamble), not on the function under test.
if [ "$HOST_FD" = 1 ]; then
    unset WS_BIND_PATH WS_BIND_ID WS_BIND_GITDIR_ID
    WS8="$T/ws8"
    mkdir "$WS8"
    ws_bind_publish "$WS8" "$(ws_identity "$WS8")"
    echo original > "$WS8/f.txt"
    ws_git init -q
    ws_bind_gitdir
    ws_git add -A
    ws_git -c user.email=matrix@example.invalid -c user.name=matrix \
        commit -qm original
    make_hostile_repo "$T/evil8" "$T/sentinel-swap8"
    rm "$WS8/f.txt"
    P8=$(pause_dir_new 8)
    CERULION_WS_PAUSE_DIR="$P8" ws_git checkout -- f.txt > "$T/out8" 2>&1 &
    PID8=$!
    wait_paused "$P8"
    mv "$WS8/.git" "$T/aside8-git"
    mv "$T/evil8/.git" "$WS8/.git" # a VALID hostile gitdir swapped INSIDE the bound workspace
    release_paused "$P8"
    if wait "$PID8"; then RC8=0; else RC8=$?; fi
    [ ! -e "$T/sentinel-swap8" ] \
        || fail "ws_git EXECUTED the swapped-in gitdir's smudge filter (gitdir check->exec seam open)"
    if [ "$RC8" -eq 0 ]; then
        [ "$(cat "$WS8/f.txt")" = original ] \
            || fail "ws_git succeeded but restored '$(cat "$WS8/f.txt")' — content from the swapped gitdir"
        echo "gitdir swap after bind: git read the ORIGINAL gitdir object (f.txt=original, no filter)"
    else
        case "$(cat "$T/out8")" in
            *"no longer carries the bound identity"*) echo "gitdir swap after bind: ws_git refused (identity gate)" ;;
            *) fail "ws_git failed for a reason other than the identity gate: $(cat "$T/out8")" ;;
        esac
    fi
    # CONTROL (anti-tautology): .git resolved BY NAME after the same swap
    # reads the hostile gitdir — its index's content AND its
    # info/attributes + local-config filter.
    rm -f "$WS8/f.txt"
    GIT_CONFIG_GLOBAL=/dev/null GIT_CONFIG_SYSTEM=/dev/null \
        git -C "$WS8" checkout -- f.txt
    [ -e "$T/sentinel-swap8" ] \
        || fail "control: by-name git did not run the swapped gitdir's filter (apparatus broken)"
    [ "$(cat "$WS8/f.txt")" = hostile ] \
        || fail "control: by-name git did not read the swapped gitdir's content (apparatus broken)"
    echo "gitdir object binding OK (ws_git read the original gitdir; by-name control read the swap and fired)"
    ARMS="31 arms"
else
    echo "selftest 8 SKIPPED on this host: /dev/fd/N is not traversable (macOS devfs), \
so ws_git resolves .git BY NAME inside the cwd-bound workspace here and this arm \
would fail by construction; the fd-bound gitdir is proven where the harness runs \
(Linux — run_matrix.sh refuses to start without it)."
    ARMS="30 of 31 arms; arm 8 skipped: no fd binding on this host"
fi

echo "== selftest 9: git_env_sweep confines git under hostile GIT_* redirection =="
# The harness launches the ENGINE's git (stage 6) outside ws_git; an
# ambient GIT_DIR / GIT_WORK_TREE / GIT_INDEX_FILE inherited from the
# caller would have pointed its commit at an EXTERNAL repository. The
# sweep must confine a plain git — started INSIDE the workspace exactly
# as the engine starts its own (`git -C <workspace>`) — to that
# workspace; the CONTROL runs the same git without the sweep and lands
# in the external repository. Both commits are --allow-empty so that a
# git operating on the wrong repository still COMMITS there (a
# "nothing to commit" refusal would hand the arm git's own error, not
# the oracle).
EXT="$T/ext9"
mkdir "$EXT"
(cd "$EXT" && plain_git init -q && echo ext > e.txt && plain_git add -A \
    && plain_git -c user.email=ext@example.invalid -c user.name=ext commit -qm ext)
EXT_HEAD=$(plain_git -C "$EXT" rev-parse HEAD)
WS9="$T/ws9"
mkdir "$WS9"
(cd "$WS9" && plain_git init -q && echo hello > f.txt && plain_git add -A \
    && plain_git -c user.email=matrix@example.invalid -c user.name=matrix commit -qm base)
WS9_HEAD=$(plain_git -C "$WS9" rev-parse HEAD)
echo changed > "$WS9/f.txt"
GIT_DIR="$EXT/.git" GIT_WORK_TREE="$EXT" GIT_INDEX_FILE="$EXT/.git/index" \
    git_env_sweep git -C "$WS9" -c core.hooksPath=/dev/null \
    -c user.email=matrix@example.invalid -c user.name=matrix \
    commit -qa --allow-empty -m swept
[ "$(plain_git -C "$EXT" rev-parse HEAD)" = "$EXT_HEAD" ] \
    || fail "the sweep let git commit to the EXTERNAL repository (GIT_DIR reached it)"
[ -z "$(plain_git -C "$EXT" status --porcelain)" ] \
    || fail "the external repository's tree changed under the swept git"
[ "$(plain_git -C "$WS9" rev-parse HEAD)" != "$WS9_HEAD" ] \
    || fail "the swept git did not commit in the workspace"
[ -z "$(plain_git -C "$WS9" status --porcelain)" ] \
    || fail "the workspace is still dirty after the swept commit"
WS9_HEAD2=$(plain_git -C "$WS9" rev-parse HEAD)
# CONTROL (anti-tautology): the same hostile environment WITHOUT the
# sweep -> the commit lands in the external repository and the
# workspace change stays uncommitted.
echo changed2 > "$WS9/f.txt"
GIT_DIR="$EXT/.git" GIT_WORK_TREE="$EXT" GIT_INDEX_FILE="$EXT/.git/index" \
    plain_git -C "$WS9" -c user.email=matrix@example.invalid -c user.name=matrix \
    commit -qa --allow-empty -m control
[ "$(plain_git -C "$EXT" rev-parse HEAD)" != "$EXT_HEAD" ] \
    || fail "control: plain git under GIT_DIR did not commit to the external repository (apparatus broken)"
[ "$(plain_git -C "$WS9" rev-parse HEAD)" = "$WS9_HEAD2" ] \
    || fail "control: plain git under GIT_DIR committed in the workspace (apparatus broken)"
# Every launch of the real verb in run_matrix.sh must go through the
# sweep (comment-stripped view): no code line may invoke "$CERULION"
# without git_env_sweep on that line, and at least one launch must exist
# (anti-vacuous).
# shellcheck disable=SC2016  # the single-quoted pattern is a regex for the literal "$CERULION"
LAUNCHES=$(printf '%s\n' "$CODE" | grep -E '"\$CERULION"' | grep -vE '^[[:space:]]*CERULION=' || true)
[ -n "$LAUNCHES" ] || fail "run_matrix.sh has no verb launch line (pin apparatus broken)"
if printf '%s\n' "$LAUNCHES" | grep -vqE 'git_env_sweep|ws_bound_launch'; then
    fail "run_matrix.sh launches the verb outside the sweep: $(printf '%s\n' "$LAUNCHES" | grep -vE 'git_env_sweep|ws_bound_launch')"
fi
echo "git_env_sweep OK (swept git confined to the workspace; control landed in the external repo; verb launches all swept)"

echo "== selftest 10: an inherited fd-binding verdict is never trusted =="
# HOST_FD is the INDEPENDENT oracle computed in the preamble (never
# the function under test, so an always-wrong probe cannot
# agree with itself). A child shell inherits the INVERTED verdict in
# every shape an environment can carry — bare, stamped by some other
# pid, and stamped with THIS shell's pid (a different process's stamp
# from the child's point of view) — and must still report the host's
# real verdict.
INVERTED=$((1 - HOST_FD))
for ambient in "$INVERTED" "12345:$INVERTED" "$$:$INVERTED"; do
    if WS_GIT_FD_BINDING="$ambient" bash -c '. ./matrix_lib.sh; ws_git_fd_binding_available'; then
        CHILD_FD=1
    else
        CHILD_FD=0
    fi
    [ "$CHILD_FD" = "$HOST_FD" ] \
        || fail "an inherited WS_GIT_FD_BINDING='$ambient' overrode the probe (host verdict $HOST_FD, child reported $CHILD_FD)"
done
echo "fd-binding cache OK (inherited verdicts ignored in all three shapes; independent host verdict $HOST_FD)"

echo "== selftest 11: the verb-launch sweep keeps the user's config + identity and still confines git =="
# The engine launch must see the CLI the way a user's shell would —
# the user's config file selectors, env-injected config and identity
# reach it — while the redirection + exec families are stripped.
EXT11="$T/ext11"
mkdir "$EXT11"
(cd "$EXT11" && plain_git init -q && echo ext > e.txt && plain_git add -A \
    && plain_git -c user.email=ext@example.invalid -c user.name=ext commit -qm ext)
EXT11_HEAD=$(plain_git -C "$EXT11" rev-parse HEAD)
WS11="$T/ws11"
mkdir "$WS11"
(cd "$WS11" && plain_git init -q && echo hello > f.txt && plain_git add -A \
    && plain_git -c user.email=matrix@example.invalid -c user.name=matrix commit -qm base)
WS11_HEAD=$(plain_git -C "$WS11" rev-parse HEAD)
echo changed > "$WS11/f.txt"
GIT_DIR="$EXT11/.git" GIT_WORK_TREE="$EXT11" GIT_INDEX_FILE="$EXT11/.git/index" \
    GIT_CONFIG_GLOBAL=/dev/null GIT_CONFIG_SYSTEM=/dev/null \
    GIT_CONFIG_COUNT=1 GIT_CONFIG_KEY_0=user.email GIT_CONFIG_VALUE_0=preserved@example.invalid \
    GIT_AUTHOR_NAME="Preserved Author" GIT_COMMITTER_NAME="Preserved Committer" \
    git_env_sweep --keep-user-config git -C "$WS11" -c core.hooksPath=/dev/null \
    commit -qa --allow-empty -m kept
[ "$(plain_git -C "$EXT11" rev-parse HEAD)" = "$EXT11_HEAD" ] \
    || fail "the keep-config sweep let git commit to the EXTERNAL repository (GIT_DIR reached it)"
[ "$(plain_git -C "$WS11" rev-parse HEAD)" != "$WS11_HEAD" ] \
    || fail "the keep-config sweep's git did not commit in the workspace"
IDENT11=$(plain_git -C "$WS11" log -1 --format='%an|%ae|%cn')
[ "$IDENT11" = "Preserved Author|preserved@example.invalid|Preserved Committer" ] \
    || fail "the keep-config sweep dropped the user's config/identity environment (commit carries '$IDENT11')"
# The EXACT split as seen by the launched command: every keep-list name
# passes, every other GIT_* — redirection, exec, and an unknown one — is
# gone; the default sweep passes nothing (the split is real).
SEEN11=$(GIT_DIR=x GIT_WORK_TREE=x GIT_INDEX_FILE=x GIT_OBJECT_DIRECTORY=x GIT_COMMON_DIR=x \
    GIT_EXEC_PATH=x GIT_TEMPLATE_DIR=x GIT_SSH_COMMAND=x GIT_ASKPASS=x GIT_EDITOR=x \
    GIT_PAGER=x GIT_EXTERNAL_DIFF=x GIT_FUTURE_UNKNOWN=x \
    GIT_CONFIG_GLOBAL=x GIT_CONFIG_SYSTEM=x GIT_CONFIG_NOSYSTEM=x \
    GIT_CONFIG_COUNT=1 GIT_CONFIG_KEY_0=x GIT_CONFIG_VALUE_0=x GIT_CONFIG_PARAMETERS=x \
    GIT_ATTR_NOSYSTEM=x GIT_AUTHOR_NAME=x GIT_AUTHOR_EMAIL=x GIT_AUTHOR_DATE=x \
    GIT_COMMITTER_NAME=x GIT_COMMITTER_EMAIL=x GIT_COMMITTER_DATE=x \
    git_env_sweep --keep-user-config env | sed -n 's/^\(GIT_[A-Za-z0-9_]*\)=.*/\1/p' | LC_ALL=C sort | tr '\n' ' ')
EXPECT11="GIT_ATTR_NOSYSTEM GIT_AUTHOR_DATE GIT_AUTHOR_EMAIL GIT_AUTHOR_NAME GIT_COMMITTER_DATE GIT_COMMITTER_EMAIL GIT_COMMITTER_NAME GIT_CONFIG_COUNT GIT_CONFIG_GLOBAL GIT_CONFIG_KEY_0 GIT_CONFIG_NOSYSTEM GIT_CONFIG_PARAMETERS GIT_CONFIG_SYSTEM GIT_CONFIG_VALUE_0 "
[ "$SEEN11" = "$EXPECT11" ] \
    || fail "the keep-config sweep passes a different GIT_* set than the stated keep-list: got '$SEEN11'"
FULL11=$(GIT_DIR=x GIT_CONFIG_GLOBAL=x GIT_AUTHOR_NAME=x git_env_sweep env | grep -c '^GIT_' || true)
[ "$FULL11" = 0 ] \
    || fail "the default sweep let $FULL11 GIT_* variable(s) through"
echo "keep-config sweep OK (confined to the workspace; identity + config preserved; exact keep-list; default sweep passes nothing)"

echo "== selftest 12: a bound launch runs in the verified workspace OBJECT, hooks ON =="
# The engine honors hooks by design, so the launch must never start it
# in a replacement tree: a hostile repo carrying a pre-commit CANARY is
# swapped in at the pathname while ws_bound_launch is parked after its
# bind + verify; the launched stand-in (a shell that records its
# physical cwd and commits with hooks ON, a hostile GIT_DIR in its
# environment) must run in the ORIGINAL object.
unset WS_BIND_PATH WS_BIND_ID WS_BIND_GITDIR_ID
WS12="$T/ws12"
mkdir "$WS12"
ws_bind_publish "$WS12" "$(ws_identity "$WS12")"
echo original > "$WS12/f.txt"
ws_git init -q
ws_bind_gitdir
ws_git add -A
ws_git -c user.email=matrix@example.invalid -c user.name=matrix \
    commit -qm original
EVIL12="$T/evil12"
mkdir "$EVIL12"
(cd "$EVIL12" && plain_git init -q && echo hostile > f.txt && plain_git add -A \
    && plain_git -c user.email=evil@example.invalid -c user.name=evil commit -qm hostile)
cat > "$EVIL12/.git/hooks/pre-commit" <<EOH
#!/bin/sh
echo pwned > "$T/sentinel-hook12"
EOH
chmod +x "$EVIL12/.git/hooks/pre-commit"
P12=$(pause_dir_new 12)
# shellcheck disable=SC2016  # the stand-in script is single-quoted on purpose: $1 expands in the launched shell
GIT_DIR="$EXT11/.git" GIT_WORK_TREE="$EXT11" GIT_INDEX_FILE="$EXT11/.git/index" \
    GIT_CONFIG_GLOBAL=/dev/null GIT_CONFIG_SYSTEM=/dev/null \
    CERULION_WS_PAUSE_DIR="$P12" ws_bound_launch sh -c '
        pwd -P > "$1"
        git -c user.email=matrix@example.invalid -c user.name=matrix \
            commit -q --allow-empty -m launched
    ' _ "$T/launch12.pwd" > "$T/out12" 2>&1 &
PID12=$!
wait_paused "$P12"
mv "$WS12" "$T/aside12"
mv "$EVIL12" "$WS12" # the reproduced shape: a hostile repo at the checked pathname
release_paused "$P12"
if wait "$PID12"; then RC12=0; else RC12=$?; fi
[ ! -e "$T/sentinel-hook12" ] \
    || fail "the bound launch EXECUTED the replacement repository's pre-commit hook (check->launch seam open)"
[ "$(plain_git -C "$EXT11" rev-parse HEAD)" = "$EXT11_HEAD" ] \
    || fail "the bound launch let its hostile GIT_DIR reach the external repository"
if [ "$RC12" -eq 0 ]; then
    [ "$(cat "$T/launch12.pwd")" = "$(cd "$T/aside12" && pwd -P)" ] \
        || fail "the launched command ran in '$(cat "$T/launch12.pwd")' rather than the ORIGINAL object"
    [ "$(plain_git -C "$T/aside12" log -1 --format=%s)" = launched ] \
        || fail "the launched commit did not land in the original object"
    [ "$(plain_git -C "$WS12" log -1 --format=%s)" = hostile ] \
        || fail "the replacement repository gained a commit"
    echo "swap after bind: the launch ran in the original object (commit landed there; hook canary silent)"
else
    case "$(cat "$T/out12")" in
        *"no longer carries the bound identity"*) echo "swap after bind: ws_bound_launch refused (identity gate)" ;;
        *) fail "the bound launch failed for a reason other than the identity gate: $(cat "$T/out12")" ;;
    esac
fi
# CONTROL (anti-tautology): the same stand-in started BY PATHNAME after
# the same swap, hooks ON -> runs in the replacement and fires its
# pre-commit canary.
(cd "$WS12" && GIT_CONFIG_GLOBAL=/dev/null GIT_CONFIG_SYSTEM=/dev/null \
    git -c user.email=matrix@example.invalid -c user.name=matrix \
    commit -q --allow-empty -m control)
[ -e "$T/sentinel-hook12" ] \
    || fail "control: the pathname launch did not run the replacement's pre-commit hook (apparatus broken)"
# Stage 6 must launch the engine this way: every "$CERULION" launch line
# goes through ws_bound_launch (comment-stripped view), the engine is
# addressed as `--workspace .`, and no `--workspace "$WS"` remains; the
# launcher itself must sweep with the keep-config mode.
if printf '%s\n' "$LAUNCHES" | grep -vq 'ws_bound_launch'; then
    fail "run_matrix.sh launches the verb outside ws_bound_launch: $(printf '%s\n' "$LAUNCHES" | grep -v ws_bound_launch)"
fi
printf '%s\n' "$LAUNCHES" | grep -q -- '--workspace \.' \
    || fail "run_matrix.sh does not address the engine as --workspace . inside the bound launch"
# shellcheck disable=SC2016  # the single-quoted pattern is the literal text --workspace "$WS"
if printf '%s\n' "$CODE" | grep -n -- '--workspace "\$WS"'; then
    fail "run_matrix.sh hands the engine the swappable pathname (--workspace \"\$WS\")"
fi
sed -n '/^ws_bound_launch()/,/^}/p' matrix_lib.sh | grep -q 'git_env_sweep --keep-user-config' \
    || fail "ws_bound_launch does not sweep with --keep-user-config"
echo "bound launch OK (ran in the original object under a swap; canary silent; control fired; stage 6 pinned)"

echo "== selftest 13: a writerless FIFO at .git is refused, never opened or waited on =="
unset WS_BIND_PATH WS_BIND_ID WS_BIND_GITDIR_ID
WS13="$T/ws13"
mkdir "$WS13"
ws_bind_publish "$WS13" "$(ws_identity "$WS13")"
echo hello > "$WS13/f.txt"
ws_git init -q
ws_bind_gitdir
ws_git add -A
ws_git -c user.email=matrix@example.invalid -c user.name=matrix commit -qm base
GOOD_GITDIR_ID=$WS_BIND_GITDIR_ID
mv "$WS13/.git" "$T/aside13-git"
mkfifo "$WS13/.git" # a writerless FIFO: a plain open would block forever
# The bind must refuse (not adopt the FIFO's identity, not block). The
# child shell models a run that published its workspace binding.
export WS_BIND_PATH WS_BIND_ID
# The timeout's status is read in the ELSE branch — after an
# `if` whose condition is false, `$?` is 0 again, so a helper that prints
# the expected refusal and then HANGS would otherwise pass as a bounded refusal.
if OUT13=$(timeout 30 bash -c '. ./matrix_lib.sh; ws_bind_gitdir' 2>&1); then
    fail "ws_bind_gitdir ACCEPTED a FIFO at .git"
else
    RC13=$?
fi
[ "$RC13" -ne 124 ] || fail "ws_bind_gitdir HUNG on the FIFO (killed by timeout)"
case "$OUT13" in
    *"not a real directory"*"FIFO"*) ;;
    *) fail "ws_bind_gitdir refused for the wrong reason: $OUT13" ;;
esac
# Every ws_git must refuse the same way, within the bound (the exported
# bindings model a run whose gitdir was replaced after its bind).
export WS_BIND_GITDIR_ID="$GOOD_GITDIR_ID"
if OUT13=$(timeout 30 bash -c '. ./matrix_lib.sh; ws_git status' 2>&1); then
    fail "ws_git EXECUTED with a FIFO at .git"
else
    RC13=$?
fi
[ "$RC13" -ne 124 ] || fail "ws_git HUNG on the FIFO at .git (killed by timeout)"
case "$OUT13" in
    *"not a real directory"*"FIFO"*) ;;
    *) fail "ws_git refused for the wrong reason: $OUT13" ;;
esac
export -n WS_BIND_PATH WS_BIND_ID WS_BIND_GITDIR_ID
rm "$WS13/.git"
mv "$T/aside13-git" "$WS13/.git"
ws_git status >/dev/null || fail "the restored gitdir must be accepted again"
echo "FIFO gitdir OK (bind + ws_git refuse without waiting; restored gitdir accepted)"

echo "== selftest 14: a symlinked .git is refused at the bind and at every ws_git =="
unset WS_BIND_PATH WS_BIND_ID WS_BIND_GITDIR_ID
WS14="$T/ws14"
mkdir "$WS14"
ws_bind_publish "$WS14" "$(ws_identity "$WS14")"
ws_git init -q
mv "$WS14/.git" "$WS14/real-git"
ln -s real-git "$WS14/.git" # a redirect: by-name stat follows it, the gate must not
if OUT14=$(ws_bind_gitdir 2>&1); then
    fail "ws_bind_gitdir ACCEPTED a symlinked .git (followed the redirect)"
fi
case "$OUT14" in
    *"not a real directory"*"symlink"*) ;;
    *) fail "ws_bind_gitdir refused for the wrong reason: $OUT14" ;;
esac
# A run bound to the REAL gitdir then refuses every ws_git while the
# symlink stands, and accepts again once it is a real directory.
rm "$WS14/.git"
mv "$WS14/real-git" "$WS14/.git"
ws_bind_gitdir
mv "$WS14/.git" "$WS14/real-git"
ln -s real-git "$WS14/.git"
if OUT14=$(ws_git status 2>&1); then
    fail "ws_git EXECUTED through a symlinked .git"
fi
case "$OUT14" in
    *"not a real directory"*"symlink"*) ;;
    *) fail "ws_git refused for the wrong reason: $OUT14" ;;
esac
rm "$WS14/.git"
mv "$WS14/real-git" "$WS14/.git"
ws_git status >/dev/null || fail "the real gitdir must be accepted again"
# The bind derives the identity from the OPENED descriptor on fd hosts —
# never from a separate pathname stat (structural, comment-stripped).
BIND_CODE=$(sed -n '/^ws_bind_gitdir()/,/^}/p' matrix_lib.sh | sed -e 's/[[:space:]]#.*$//' -e '/^[[:space:]]*#/d')
printf '%s\n' "$BIND_CODE" | grep -q 'ws_open_gitdir_in_cwd' \
    || fail "ws_bind_gitdir does not open the gitdir through the type-gated open"
# shellcheck disable=SC2016  # the single-quoted pattern is the literal source text
printf '%s\n' "$BIND_CODE" | grep -q 'ws_identity "/dev/fd/\$WS_BOUND_GFD"' \
    || fail "ws_bind_gitdir does not derive the identity from the opened descriptor on fd hosts"
echo "symlinked gitdir OK (bind + ws_git refuse; real gitdir accepted; descriptor-derived bind pinned)"

echo "== selftest 15: analysis-input descendants are bound and re-verified =="
unset WS_BIND_PATH WS_BIND_ID WS_BIND_GITDIR_ID
WS15="$T/ws15"
mkdir -p "$WS15/src" "$WS15/build/pkg"
ws_bind_publish "$WS15" "$(ws_identity "$WS15")"
SRC15=$(ws_bind_descendant src) || fail "binding a real descendant directory must succeed"
BLD15=$(ws_bind_descendant build/pkg) || fail "binding a nested descendant must succeed"
require_ws_descendant src "$SRC15" "the selftest happy arm" \
    || fail "an untouched descendant must pass"
require_ws_descendant build/pkg "$BLD15" "the selftest happy arm" \
    || fail "an untouched nested descendant must pass"
mv "$WS15/src" "$T/aside15-src"
mkdir "$WS15/src" # a swapped-in replacement at the same pathname
if require_ws_descendant src "$SRC15" "the selftest swap arm" 2>/dev/null; then
    fail "a swapped descendant was ACCEPTED"
fi
rmdir "$WS15/src"
ln -s "$T/aside15-src" "$WS15/src" # a symlinked descendant must not bind
if ws_bind_descendant src >/dev/null 2>&1; then
    fail "a symlinked descendant was BOUND"
fi
# run_matrix.sh binds both analysis inputs and invokes the prover ONLY
# through `analyze`, which re-verifies them (comment-stripped view).
grep -qE 'ws_bind_descendant src' "$CODE_FILE" \
    || fail "run_matrix.sh does not bind src"
grep -qE 'ws_bind_descendant build/migrate_fixture_pkg' "$CODE_FILE" \
    || fail "run_matrix.sh does not bind the compile-database directory"
# shellcheck disable=SC2016  # the single-quoted pattern is the literal source text
if printf '%s\n' "$CODE" | grep -nE '"\$(TOOL|mut)" -p' | grep -vE 'analyze '; then
    fail "run_matrix.sh invokes the prover outside analyze"
fi
sed -n '/^analyze()/,/^}/p' run_matrix.sh | grep -q 'require_analysis_inputs' \
    || fail "analyze does not re-verify the analysis inputs"
sed -n '/^require_analysis_inputs()/,/^}/p' run_matrix.sh | grep -q 'require_ws_descendant src' \
    || fail "require_analysis_inputs does not re-verify src"
echo "descendant binding OK (swap refused; symlink not bound; run_matrix pinned)"

echo "== selftest 16: a symlink to the same inode is refused by every pathname check =="
unset WS_BIND_PATH WS_BIND_ID WS_BIND_GITDIR_ID
WS16="$T/ws16"
mkdir -p "$WS16/src"
ws_bind_publish "$WS16" "$(ws_identity "$WS16")"
ws_git init -q
ws_bind_gitdir
SRC16=$(ws_bind_descendant src) || fail "binding src must succeed"
require_ws_descendant src "$SRC16" "the selftest happy arm" || fail "an untouched descendant must pass"
# The bound directory is moved aside and its pathname replaced by a
# symlink to it: `stat -L` reports the SAME identity, so an identity-only
# check accepted a pathname an actor now controls.
mv "$WS16/src" "$T/aside16-src"
ln -s "$T/aside16-src" "$WS16/src"
[ "$(ws_identity "$WS16/src")" = "$SRC16" ] || fail "apparatus: the symlink must resolve to the bound identity"
if OUT16=$(require_ws_descendant src "$SRC16" "the selftest link arm" 2>&1); then
    fail "require_ws_descendant ACCEPTED a symlink to the bound inode"
fi
case "$OUT16" in
    *"not a real directory"*"symlink"*) ;;
    *) fail "the descendant check refused for the wrong reason: $OUT16" ;;
esac
rm "$WS16/src"
mv "$T/aside16-src" "$WS16/src"
# The same for the workspace and gitdir pathname checks.
mv "$WS16" "$T/aside16-ws"
ln -s "$T/aside16-ws" "$WS16"
if OUT16=$(require_ws_and_gitdir "$WS16" "$WS_BIND_ID" "$WS_BIND_GITDIR_ID" "the selftest ws link arm" 2>&1); then
    fail "require_ws_and_gitdir ACCEPTED a symlinked workspace pathname to the bound inode"
fi
case "$OUT16" in
    *"not a real directory"*"symlink"*) ;;
    *) fail "the workspace check refused for the wrong reason: $OUT16" ;;
esac
rm "$WS16"
mv "$T/aside16-ws" "$WS16"
require_ws_and_gitdir "$WS16" "$WS_BIND_ID" "$WS_BIND_GITDIR_ID" "the selftest restored arm" \
    || fail "the restored workspace must pass again"
echo "symlink-to-inode OK (descendant, workspace and gitdir pathname checks refuse; restored passes)"

echo "== selftest 17: the analysis inputs are bound by content =="
WS17="$T/ws17"
mkdir -p "$WS17/src/pkg" "$WS17/build/pkg"
printf 'int a;\n' > "$WS17/src/pkg/a.cpp"
printf 'int b;\n' > "$WS17/src/pkg/b.cpp"
printf '[]\n' > "$WS17/build/pkg/compile_commands.json"
D17=$(ws_tree_digest "$WS17/src") || fail "digesting a tree must succeed"
C17=$(ws_sha256 < "$WS17/build/pkg/compile_commands.json")
[ "$(ws_tree_digest "$WS17/src")" = "$D17" ] || fail "an untouched tree must digest identically"
require_ws_tree_digest "$WS17/src" "$D17" "the selftest happy arm" || fail "an untouched tree must pass"
require_ws_file_digest "$WS17/build/pkg/compile_commands.json" "$C17" "the selftest happy arm" \
    || fail "an untouched compile db must pass"
ID17=$(ws_identity "$WS17/src")
printf 'int a; /* substituted */\n' > "$WS17/src/pkg/a.cpp" # same file, same dir identity, new bytes
[ "$(ws_identity "$WS17/src")" = "$ID17" ] || fail "apparatus: the directory identity must be unchanged"
if require_ws_tree_digest "$WS17/src" "$D17" "the selftest substitution arm" 2>/dev/null; then
    fail "a substituted source file under an unchanged directory identity was ACCEPTED"
fi
printf '[{"x":1}]\n' > "$WS17/build/pkg/compile_commands.json"
if require_ws_file_digest "$WS17/build/pkg/compile_commands.json" "$C17" "the selftest substitution arm" 2>/dev/null; then
    fail "a substituted compile db was ACCEPTED"
fi
# run_matrix.sh captures both digests before analysis and re-verifies
# them after stages 3, 3b, 4 and 5 (comment-stripped view).
grep -qE '^SRC_DIGEST=\$\(ws_tree_digest' "$CODE_FILE" \
    || fail "run_matrix.sh does not capture the source digest"
grep -qE '^CCDB_DIGEST=\$\(ws_file_sha256' "$CODE_FILE" \
    || fail "run_matrix.sh does not capture the compile-db digest"
for stage in 'stage 3 ' 'stage 3b ' 'stage 4 ' 'stage 5 '; do
    printf '%s\n' "$CODE" | grep -qE "require_analysis_digest \"$stage" \
        || fail "run_matrix.sh does not re-verify the input digests after $stage"
done
sed -n '/^require_analysis_digest()/,/^}/p' run_matrix.sh | grep -q 'require_ws_tree_digest' \
    || fail "require_analysis_digest does not check the source tree"
# The compile database check is pinned separately.
sed -n '/^require_analysis_digest()/,/^}/p' run_matrix.sh | grep -q 'require_ws_file_digest' \
    || fail "require_analysis_digest does not check the compile database"
grep -qE '^CCDB_DIGEST=\$\(ws_file_sha256' "$CODE_FILE" \
    || fail "run_matrix.sh captures the compile-db digest without the type-gated bounded reader"
# The r19 pin above already requires the tree check; the compile-db pin is
# distinct so dropping either call fails with its own message.
echo "content bind OK (file substitution under an unchanged identity refused; run_matrix re-verifies after every analysis stage)"

echo "== selftest 18: a symlinked INTERMEDIATE component is refused =="
unset WS_BIND_PATH WS_BIND_ID WS_BIND_GITDIR_ID
WS18="$T/ws18"
mkdir -p "$WS18/build/pkg"
ws_bind_publish "$WS18" "$(ws_identity "$WS18")"
ws_git init -q
ws_bind_gitdir
B18=$(ws_bind_descendant build/pkg) || fail "binding through real directories must succeed"
require_ws_descendant build/pkg "$B18" "the selftest happy arm" \
    || fail "the real-directory chain must pass (control)"
# The bound tree is moved aside and `build` linked to it: the FINAL
# component build/pkg still resolves to the bound inode.
mv "$WS18/build" "$T/aside18-build"
ln -s "$T/aside18-build" "$WS18/build"
[ "$(ws_identity "$WS18/build/pkg")" = "$B18" ] || fail "apparatus: the final component must still carry the bound identity"
if OUT18=$(require_ws_descendant build/pkg "$B18" "the selftest link arm" 2>&1); then
    fail "require_ws_descendant ACCEPTED a descendant reached through a symlinked intermediate component"
fi
case "$OUT18" in
    *"component 'build'"*"symlink"*) ;;
    *) fail "the recheck refused for the wrong reason: $OUT18" ;;
esac
if ws_bind_descendant build/pkg >/dev/null 2>&1; then
    fail "ws_bind_descendant ACCEPTED a symlinked intermediate component"
fi
if ws_bind_descendant '../ws18/build/pkg' >/dev/null 2>&1; then
    fail "ws_bind_descendant ACCEPTED a '..' component"
fi
rm "$WS18/build"
mv "$T/aside18-build" "$WS18/build"
require_ws_descendant build/pkg "$B18" "the selftest restored arm" \
    || fail "the restored real chain must pass again"
echo "intermediate-symlink OK (bind + recheck refuse a linked component; real chain passes)"

echo "== selftest 19: symlink targets are digested byte-exact =="
T19A="$T/t19a"
T19B="$T/t19b"
T19C="$T/t19c"
mkdir "$T19A" "$T19B" "$T19C"
touch "$T19A/safe.cpp" "$T19B/safe.cpp" "$T19C/safe.cpp"
ln -s safe.cpp "$T19A/link"
ln -s $'safe.cpp\n' "$T19B/link" # the target differs ONLY by a trailing newline
ln -s safe.cpp "$T19C/link"
[ "$(readlink -n -- "$T19B/link" | od -An -v -tx1 | tr -d ' \n' | tail -c 2)" = "0a" ] \
    || fail "apparatus: the newline-terminated target must be readable byte-exact"
D19A=$(ws_tree_digest "$T19A")
D19B=$(ws_tree_digest "$T19B")
D19C=$(ws_tree_digest "$T19C")
[ "$D19A" != "$D19B" ] \
    || fail "two link targets differing only by a trailing newline digested IDENTICALLY"
[ "$D19A" = "$D19C" ] || fail "identical trees must digest identically (control)"
echo "link-target bytes OK (a trailing newline changes the digest; identical trees agree)"

echo "== selftest 20: a FIFO in place of the compile database is refused, never waited on =="
WS20="$T/ws20"
mkdir -p "$WS20/build/pkg"
printf '[]\n' > "$WS20/build/pkg/compile_commands.json"
C20=$(ws_file_sha256 "$WS20/build/pkg/compile_commands.json") || fail "hashing a regular file must succeed"
require_ws_file_digest "$WS20/build/pkg/compile_commands.json" "$C20" "the selftest happy arm" \
    || fail "an untouched compile db must pass (control)"
rm "$WS20/build/pkg/compile_commands.json"
mkfifo "$WS20/build/pkg/compile_commands.json" # writerless: a plain read would block forever
# The capture refuses (bounded, not hung).
# shellcheck disable=SC2016  # $1 expands in the child shell
if OUT20=$(timeout 30 bash -c '. ./matrix_lib.sh; ws_file_sha256 "$1"' _ "$WS20/build/pkg/compile_commands.json" 2>&1); then
    fail "ws_file_sha256 ACCEPTED a FIFO"
else
    RC20=$?
fi
[ "$RC20" -ne 124 ] || fail "ws_file_sha256 HUNG on the FIFO (killed by timeout)"
case "$OUT20" in
    *"not a regular file"*"FIFO"*) ;;
    *) fail "the capture refused for the wrong reason: $OUT20" ;;
esac
# The recheck refuses the same way, inside the bound.
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
if OUT20=$(timeout 30 bash -c '. ./matrix_lib.sh; require_ws_file_digest "$1" "$2" "the selftest FIFO arm"' _ "$WS20/build/pkg/compile_commands.json" "$C20" 2>&1); then
    fail "require_ws_file_digest ACCEPTED a FIFO"
else
    RC20=$?
fi
[ "$RC20" -ne 124 ] || fail "require_ws_file_digest HUNG on the FIFO (killed by timeout)"
case "$OUT20" in
    *"not a regular file"*"FIFO"*) ;;
    *) fail "the recheck refused for the wrong reason: $OUT20" ;;
esac
echo "FIFO compile db OK (capture + recheck refuse without waiting)"

echo "== selftest 21: the prover analyzes an immutable verified copy =="
WS21="$T/ws21"
mkdir -p "$WS21/src/pkg/src" "$WS21/build/pkg"
printf 'int approved_bytes;\n' > "$WS21/src/pkg/src/x.cpp"
printf '[{"directory": "%s", "command": "clang++ -c %s", "file": "%s"}]\n' \
    "$WS21/build/pkg" "$WS21/src/pkg/src/x.cpp" "$WS21/src/pkg/src/x.cpp" \
    > "$WS21/build/pkg/compile_commands.json"
D21=$(ws_tree_digest "$WS21/src")
C21=$(ws_file_sha256 "$WS21/build/pkg/compile_commands.json")
SNAP21=$(mktemp -d "$T/snap21.XXXXXX")
ws_snapshot_inputs "$WS21/src" "$D21" "$WS21/build/pkg/compile_commands.json" "$C21" "$SNAP21" \
    || fail "snapshotting approved inputs must succeed"
[ "$(cat "$SNAP21/src/pkg/src/x.cpp")" = "int approved_bytes;" ] || fail "the snapshot must hold the approved bytes"
ws_rewrite_ccdb_paths "$SNAP21/ccdb/compile_commands.json" "$WS21/src" "$SNAP21/src" || fail "rewriting the compile db copy must succeed"
grep -q "\"file\": \"$SNAP21/src/pkg/src/x.cpp\"" "$SNAP21/ccdb/compile_commands.json" \
    || fail "the compile db copy must point at the snapshot source"
grep -q "\"directory\": \"$WS21/build/pkg\"" "$SNAP21/ccdb/compile_commands.json" \
    || fail "the compile db copy must keep the build directory"
# A stand-in prover: parks on the handshake, then emits the bytes of the
# TU it was pointed at (its last argument) as its "analysis".
PROVER21="$T/prover21.sh"
cat > "$PROVER21" <<'EOP'
#!/usr/bin/env bash
echo paused > "$CERULION_WS_PAUSE_DIR/ready"
read -r _ < "$CERULION_WS_PAUSE_DIR/go" || true
for last; do :; done
printf '{"analyzed": "%s"}\n' "$(cat "$last")"
EOP
chmod +x "$PROVER21"
P21=$(pause_dir_new 21)
CERULION_WS_PAUSE_DIR="$P21" ws_analyze_snapshot "$SNAP21" "$PROVER21" pkg/src/x.cpp > "$T/out21.json" 2>&1 &
PID21=$!
wait_paused "$P21" # the prover is running; now the LIVE source is swapped
mv "$WS21/src/pkg/src/x.cpp" "$T/aside21.cpp"
printf 'int MARKER_SUBSTITUTED;\n' > "$WS21/src/pkg/src/x.cpp"
release_paused "$P21"
wait "$PID21" || fail "the stand-in prover must succeed: $(cat "$T/out21.json")"
mv "$T/aside21.cpp" "$WS21/src/pkg/src/x.cpp" # restored before any post-stage check
require_ws_tree_digest "$WS21/src" "$D21" "the selftest restored arm" \
    || fail "apparatus: the restored live tree must digest as approved (the swap is invisible to the post-check — the reproduced hole)"
grep -q 'MARKER_SUBSTITUTED' "$T/out21.json" && fail "the analysis carried the SUBSTITUTED bytes: $(cat "$T/out21.json")"
grep -q 'int approved_bytes;' "$T/out21.json" \
    || fail "the analysis must carry the APPROVED bytes: $(cat "$T/out21.json")"
ws_map_snapshot_paths "$T/out21.json" "$SNAP21/src" "$WS21/src" || fail "mapping paths back must succeed"
# Refusals: a symlink in the live tree, and a copy that does not match the
# approved digest, are never snapshotted silently.
SNAP21B=$(mktemp -d "$T/snap21b.XXXXXX")
if ws_snapshot_inputs "$WS21/src" "0000" "$WS21/build/pkg/compile_commands.json" "$C21" "$SNAP21B" 2>/dev/null; then
    fail "a snapshot that does not match the approved digest was ACCEPTED"
fi
SNAP21C=$(mktemp -d "$T/snap21c.XXXXXX")
ln -s x.cpp "$WS21/src/pkg/src/link.cpp"
if ws_snapshot_inputs "$WS21/src" "$D21" "$WS21/build/pkg/compile_commands.json" "$C21" "$SNAP21C" 2>/dev/null; then
    fail "a live tree holding a symlink was snapshotted"
fi
rm "$WS21/src/pkg/src/link.cpp"
# run_matrix.sh snapshots the inputs, verifies the copy, and runs the
# prover ONLY through ws_analyze_snapshot (comment-stripped view).
grep -qE '^SNAP_ROOT=\$\(mktemp -d' "$CODE_FILE" \
    || fail "run_matrix.sh does not mint a fresh private snapshot root"
# shellcheck disable=SC2016  # the single-quoted pattern is the literal source text
grep -qE '^ws_snapshot_inputs "\$WS/src" "\$SRC_DIGEST"' "$CODE_FILE" \
    || fail "run_matrix.sh does not snapshot the approved inputs"
grep -qE '^ws_rewrite_ccdb_paths ' "$CODE_FILE" \
    || fail "run_matrix.sh does not rewrite the compile db copy's paths"
# shellcheck disable=SC2016  # the single-quoted pattern is the literal source text
sed -n '/^analyze()/,/^}/p' run_matrix.sh | grep -q 'ws_analyze_snapshot "\$SNAP_ROOT"' \
    || fail "analyze does not run the prover against the snapshot"
# shellcheck disable=SC2016  # the single-quoted pattern is the literal source text
if printf '%s\n' "$CODE" | grep -nE '"\$(TOOL|mut)" -p|src-root="\$WS/src"'; then
    fail "run_matrix.sh points the prover at the LIVE tree"
fi
# shellcheck disable=SC2016  # the single-quoted pattern is the literal source text
sed -n '/^ws_analyze_snapshot()/,/^}/p' matrix_lib.sh | grep -q '"\$1/src/\$3"' \
    || fail "ws_analyze_snapshot does not hand the prover the snapshot copy of the TU"
echo "immutable copy OK (mid-run swap invisible to the post-check, yet the analysis carries the approved bytes; run_matrix pinned)"

echo "== selftest 22: the snapshot is sealed read-only and re-digests ITSELF after each stage =="
WS22="$T/ws22"
mkdir -p "$WS22/src/pkg/src" "$WS22/build/pkg"
printf 'int approved_bytes;\n' > "$WS22/src/pkg/src/x.cpp"
printf '[{"directory": "%s", "command": "clang++ -c %s", "file": "%s"}]\n' \
    "$WS22/build/pkg" "$WS22/src/pkg/src/x.cpp" "$WS22/src/pkg/src/x.cpp" \
    > "$WS22/build/pkg/compile_commands.json"
D22=$(ws_tree_digest "$WS22/src")
C22=$(ws_file_sha256 "$WS22/build/pkg/compile_commands.json")
SNAP22=$(mktemp -d "$T/snap22.XXXXXX")
ws_snapshot_inputs "$WS22/src" "$D22" "$WS22/build/pkg/compile_commands.json" "$C22" "$SNAP22" \
    || fail "snapshotting approved inputs must succeed"
ws_rewrite_ccdb_paths "$SNAP22/ccdb/compile_commands.json" "$WS22/src" "$SNAP22/src" || fail "rewriting the compile db copy must succeed"
R22=$(ws_file_sha256 "$SNAP22/ccdb/compile_commands.json")
if require_ws_snapshot_sealed "$SNAP22" "the unsealed control" 2>/dev/null; then
    fail "apparatus: an UNSEALED snapshot passed the seal check"
fi
ws_snapshot_seal "$SNAP22" || fail "sealing the snapshot must succeed"
require_ws_snapshot_intact "$SNAP22" "$D22" "$R22" "the sealed snapshot" \
    || fail "a sealed, untouched snapshot must be intact"
# (i) PREVENTION for an unprivileged writer: nothing under the root can be
# written, created, renamed or replaced — files AND directories (root
# ignores modes, so that half is not observable as root).
if [ "$(id -u)" = 0 ]; then
    echo "  (running as root: the unprivileged-writer prevention half is not observable — skipped)"
else
    if (printf 'int MARKER_SUBSTITUTED;\n' > "$SNAP22/src/pkg/src/x.cpp") 2>/dev/null; then
        fail "a write into a sealed snapshot file SUCCEEDED"
    fi
    printf 'int MARKER_SUBSTITUTED;\n' > "$T/marker22.cpp"
    if mv -f "$T/marker22.cpp" "$SNAP22/src/pkg/src/x.cpp" 2>/dev/null; then
        fail "a rename over a sealed snapshot file SUCCEEDED (the directory is still writable)"
    fi
    if touch "$SNAP22/src/pkg/src/new.cpp" 2>/dev/null || touch "$SNAP22/new" 2>/dev/null; then
        fail "an entry was CREATED inside the sealed snapshot"
    fi
    [ "$(cat "$SNAP22/src/pkg/src/x.cpp")" = "int approved_bytes;" ] || fail "apparatus: the sealed file changed"
fi
# (ii) DETECTION against a same-UID writer who chmods the seal away —
# the worst-case timing against the snapshot: the TU is replaced inside the
# snapshot while the parked prover runs. The analysis DOES carry the
# substituted bytes (prevention is gone: the accepted residual) and the
# stage is REFUSED by the snapshot's own post-stage checks — first by the
# seal check, and, once the attacker re-seals, by the snapshot digest.
PROVER22="$T/prover22.sh"
cat > "$PROVER22" <<'EOP'
#!/usr/bin/env bash
echo paused > "$CERULION_WS_PAUSE_DIR/ready"
read -r _ < "$CERULION_WS_PAUSE_DIR/go" || true
for last; do :; done
printf '{"analyzed": "%s"}\n' "$(cat "$last")"
EOP
chmod +x "$PROVER22"
P22=$(pause_dir_new 22)
CERULION_WS_PAUSE_DIR="$P22" ws_analyze_snapshot "$SNAP22" "$PROVER22" pkg/src/x.cpp > "$T/out22.json" 2>&1 &
PID22=$!
wait_paused "$P22" # the prover is running; the same-UID attacker unseals the directory and swaps the TU
chmod u+w "$SNAP22/src/pkg/src" || fail "apparatus: a same-UID writer must be able to chmod the snapshot directory"
printf 'int MARKER_SUBSTITUTED;\n' > "$T/marker22b.cpp"
mv -f "$T/marker22b.cpp" "$SNAP22/src/pkg/src/x.cpp" || fail "apparatus: a same-UID writer must be able to replace the TU once unsealed"
release_paused "$P22"
wait "$PID22" || fail "the stand-in prover must succeed: $(cat "$T/out22.json")"
grep -q 'MARKER_SUBSTITUTED' "$T/out22.json" \
    || fail "apparatus: the swap did not reach the prover, so there is nothing to detect: $(cat "$T/out22.json")"
if require_ws_snapshot_intact "$SNAP22" "$D22" "$R22" "the post-stage check" 2> "$T/err22"; then
    fail "an in-snapshot substitution was ACCEPTED by the post-stage snapshot check"
fi
grep -q 'not sealed' "$T/err22" || fail "the refusal must name the broken seal: $(cat "$T/err22")"
chmod a-w "$SNAP22/src/pkg/src/x.cpp" "$SNAP22/src/pkg/src" # the attacker re-seals; the bytes stay substituted
require_ws_snapshot_sealed "$SNAP22" "the re-sealed snapshot" \
    || fail "apparatus: the re-sealed snapshot must pass the seal check"
if require_ws_snapshot_intact "$SNAP22" "$D22" "$R22" "the post-stage check (re-sealed)" 2> "$T/err22b"; then
    fail "a re-sealed snapshot with SUBSTITUTED bytes was ACCEPTED"
fi
grep -q 'snapshot sources' "$T/err22b" || fail "the refusal must name the snapshot sources digest: $(cat "$T/err22b")"
# run_matrix.sh: rewritten-ccdb digest captured, seal applied BEFORE the
# first prover run, seal re-checked in analyze, snapshot re-digested in
# require_analysis_digest, unsealed before deletion (comment-stripped view).
# shellcheck disable=SC2016  # the single-quoted pattern is the literal source text
grep -qE '^SNAP_CCDB_DIGEST=\$\(ws_file_sha256 "\$SNAP_ROOT/ccdb/compile_commands.json"' "$CODE_FILE" \
    || fail "run_matrix.sh does not capture the rewritten compile db's digest"
# shellcheck disable=SC2016  # the single-quoted pattern is the literal source text
grep -qE '^ws_snapshot_seal "\$SNAP_ROOT"' "$CODE_FILE" \
    || fail "run_matrix.sh does not seal the snapshot"
# shellcheck disable=SC2016  # the single-quoted pattern is the literal source text
seal_line=$(printf '%s\n' "$CODE" | grep -nE '^ws_snapshot_seal "\$SNAP_ROOT"' | cut -d: -f1)
first_analyze=$(printf '%s\n' "$CODE" | grep -nE '^ *analyze "' | head -1 | cut -d: -f1)
[ -n "$seal_line" ] && [ -n "$first_analyze" ] && [ "$seal_line" -lt "$first_analyze" ] \
    || fail "run_matrix.sh does not seal the snapshot BEFORE the first prover run"
# shellcheck disable=SC2016  # the single-quoted pattern is the literal source text
sed -n '/^analyze()/,/^}/p' run_matrix.sh | grep -q 'require_ws_snapshot_sealed "\$SNAP_ROOT"' \
    || fail "analyze does not re-check the seal before the prover runs"
# shellcheck disable=SC2016  # the single-quoted pattern is the literal source text
sed -n '/^require_analysis_digest()/,/^}/p' run_matrix.sh | grep -q 'require_ws_snapshot_intact "\$SNAP_ROOT" "\$SRC_DIGEST" "\$SNAP_CCDB_DIGEST"' \
    || fail "require_analysis_digest does not re-digest the snapshot itself"
sed -n '/^cleanup_snapshot()/,/^}/p' run_matrix.sh | grep -q 'ws_snapshot_unseal' \
    || fail "cleanup_snapshot does not unseal before deleting (a sealed root would be left behind)"
ws_snapshot_unseal "$SNAP22"
echo "sealed snapshot OK (unprivileged writes/renames/creates refused; a same-UID swap is detected by the seal check and, re-sealed, by the snapshot digest; run_matrix pinned)"

echo "== selftest 23: prover RESULTS live in a sealed private dir, digested over the exact set, re-verified by every consumer =="
R23=$(mktemp -d "$T/results23.XXXXXX")
mkdir -m 0700 "$R23/stage3"
# A stand-in prover emits an oracle-compatible result for the TU it was
# handed; a stand-in consumer "applies" every result's edit (appends it to
# applied23) — the shape of assert_matrix.py + the stage-3b apply.
PROVER23="$T/prover23.sh"
cat > "$PROVER23" <<'EOP'
#!/usr/bin/env bash
printf '{"tu": "%s", "edit": "PROVER_OUTPUT"}\n' "$1"
EOP
chmod +x "$PROVER23"
ws_capture_result "$R23/stage3/a.cpp.json" "$PROVER23" a.cpp || fail "capturing a result must succeed"
ws_capture_result "$R23/stage3/b.cpp.json" "$PROVER23" b.cpp || fail "capturing a result must succeed"
grep -q '"edit": "PROVER_OUTPUT"' "$R23/stage3/a.cpp.json" || fail "apparatus: the captured result must carry the prover's edit"
printf 'pre-placed\n' > "$R23/stage3/c.cpp.json"
if ws_capture_result "$R23/stage3/c.cpp.json" "$PROVER23" c.cpp 2>/dev/null; then
    fail "a PRE-PLACED result file was overwritten by the capture (create-new violated)"
fi
[ "$(cat "$R23/stage3/c.cpp.json")" = "pre-placed" ] || fail "apparatus: the pre-placed file changed"
rm "$R23/stage3/c.cpp.json"
ws_results_seal "$R23/stage3" || fail "sealing the results must succeed"
SET23=$'a.cpp.json\nb.cpp.json'
DIG23=$(ws_results_digest "$R23/stage3" "$SET23") || fail "digesting the sealed results must succeed"
[ "$DIG23" = "$(ws_results_digest "$R23/stage3" "$SET23")" ] || fail "apparatus: the results digest must be stable"
apply23() {
    local f
    for f in "$R23/stage3"/*.json; do
        python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["edit"])' "$f" >> "$T/applied23"
    done
}
consume23() {
    # run_matrix's consume_results shape: verify immediately before the
    # read, read, verify immediately after.
    require_ws_results_intact "$R23/stage3" "$DIG23" "$SET23" "the stand-in consumer, pre-read" || return 2
    apply23
    require_ws_results_intact "$R23/stage3" "$DIG23" "$SET23" "the stand-in consumer, post-read" || return 2
}
: > "$T/applied23"
consume23 || fail "untouched sealed results must be consumable (the original marker case)"
# `sort -u` would collapse the TWO expected lines into
# one, so a stand-in consumer that processed only a.cpp.json and skipped
# b.cpp.json would pass this check. Multiplicity is the property — assert the
# count and every line, never a deduplicated set.
APPLIED23=$(cat "$T/applied23")
[ "$(printf '%s\n' "$APPLIED23" | grep -c .)" = 2 ] \
    || fail "apparatus: the consumer must apply BOTH results, got: $APPLIED23"
[ "$APPLIED23" = "$(printf 'PROVER_OUTPUT\nPROVER_OUTPUT')" ] \
    || fail "apparatus: the consumer must apply exactly the prover's edits: $APPLIED23"
# (i) PREVENTION for an unprivileged writer (root ignores modes).
if [ "$(id -u)" = 0 ]; then
    echo "  (running as root: the unprivileged-writer prevention half is not observable — skipped)"
else
    if (printf '{"edit": "FORGED_OUTPUT"}\n' > "$R23/stage3/a.cpp.json") 2>/dev/null; then
        fail "a write into a sealed result SUCCEEDED"
    fi
    printf '{"edit": "FORGED_OUTPUT"}\n' > "$T/forged23.json"
    if mv -f "$T/forged23.json" "$R23/stage3/a.cpp.json" 2>/dev/null; then
        fail "a rename over a sealed result SUCCEEDED (the directory is still writable)"
    fi
    if touch "$R23/stage3/extra.json" 2>/dev/null; then
        fail "an entry was CREATED inside the sealed results dir"
    fi
fi
# (ii) DETECTION — the worst-case timing: a same-UID writer forges an
# oracle-compatible result between the prover's exit and the consumer,
# chmodding the seal away and back so the seal check alone would pass.
# The consumer gate must refuse naming the digest, and NOTHING is applied.
chmod u+w "$R23/stage3" || fail "apparatus: a same-UID writer must be able to chmod the results dir"
printf '{"tu": "a.cpp", "edit": "FORGED_OUTPUT"}\n' > "$T/forged23b.json"
mv -f "$T/forged23b.json" "$R23/stage3/a.cpp.json" || fail "apparatus: a same-UID writer must be able to replace the result once unsealed"
chmod a-w "$R23/stage3/a.cpp.json" "$R23/stage3"
require_ws_snapshot_sealed "$R23/stage3" "the re-sealed results" || fail "apparatus: the re-sealed results must pass the seal check alone"
: > "$T/applied23"
if consume23 2> "$T/err23"; then
    fail "a FORGED result was ACCEPTED by the consumer gate"
fi
grep -q 'changed since they were sealed' "$T/err23" || fail "the refusal must name the digest mismatch: $(cat "$T/err23")"
if [ -s "$T/applied23" ]; then
    fail "the FORGED result was APPLIED: $(cat "$T/applied23")"
fi
# An EXTRA result beside the expected set, and a MISSING one, are refused
# as a set mismatch — a consumer never globs up a forgery.
chmod u+w "$R23/stage3"
printf '{"edit": "FORGED_OUTPUT"}\n' > "$R23/stage3/zz.cpp.json"
chmod a-w "$R23/stage3/zz.cpp.json" "$R23/stage3"
if require_ws_results_intact "$R23/stage3" "$DIG23" "$SET23" "the extra-file check" 2> "$T/err23c"; then
    fail "an EXTRA result file was ACCEPTED"
fi
grep -q 'not the expected set' "$T/err23c" || fail "the refusal must name the set mismatch: $(cat "$T/err23c")"
chmod u+w "$R23/stage3"
rm -f "$R23/stage3/zz.cpp.json" "$R23/stage3/b.cpp.json"
chmod a-w "$R23/stage3"
if require_ws_results_intact "$R23/stage3" "$DIG23" "$SET23" "the missing-file check" 2> "$T/err23d"; then
    fail "a MISSING result file was ACCEPTED"
fi
grep -q 'not the expected set' "$T/err23d" || fail "the refusal must name the set mismatch: $(cat "$T/err23d")"
# A writer who did not bother to re-seal is refused by the seal check
# before anything is read.
chmod u+w "$R23/stage3"
if require_ws_results_intact "$R23/stage3" "$DIG23" "$SET23" "the unsealed check" 2> "$T/err23e"; then
    fail "an UNSEALED results dir was ACCEPTED"
fi
grep -q 'not sealed' "$T/err23e" || fail "the refusal must name the broken seal: $(cat "$T/err23e")"
# run_matrix.sh (comment-stripped view; continuation lines joined): a
# fresh private results root; stage 3 seals + digests before its first
# consumer; consume_results re-binds the root and verifies before AND
# after the read; every consumer runs through it and reads only sealed
# paths; $OUTDIR is an export nobody reads; no result is captured into
# $MATRIX_TMP any more; the root is unsealed for deletion.
JOINED_FILE="$T/run_matrix.joined"
# The same bash-faithful join as the arm-27 walk, fed the RAW file
# for the same reason that walk is. These greps are existence checks, so a
# wrong FUSION can glue a compliant `consume_results` prefix onto an
# unrelated consumer line and read as compliant — the same false-pass class,
# one walk over. Feeding it `$CODE_FILE` (comments already stripped at the
# CODE assignment above) would have left exactly that hole open here: the
# join's comment handling would be inert, and a comment spliced into a
# continuation would still fuse the commands on either side. `$CODE_FILE`
# stays as it is for the line-oriented pins that want the stripped view;
# this walk strips its comments AFTER joining, inside the function.
# VERIFIED on today's run_matrix.sh, and stated as what was actually checked:
# the new join KEEPS the continuation's leading whitespace where the old
# `sed -e 's/\\\n *//'` deleted it, so joined lines genuinely differ and a
# whitespace-squeezed set compare is blind to exactly that. What was run is
# the consumers themselves: all six `consume_results` greps match against the
# new view (the `" +(.* )?` in the pattern absorbs the retained indentation at
# the two points where continuations land), and the forbidden-path grep below
# finds nothing in either view.
join_logical_lines run_matrix.sh > "$JOINED_FILE"
grep -qE '^RESULTS_ROOT=\$\(mktemp -d' "$CODE_FILE" \
    || fail "run_matrix.sh does not mint a fresh private results root"
# shellcheck disable=SC2016  # the single-quoted pattern is the literal source text
grep -qE '^ws_results_seal "\$RESULTS_ROOT/stage3"' "$CODE_FILE" \
    || fail "run_matrix.sh does not seal the stage-3 results"
# shellcheck disable=SC2016  # the single-quoted pattern is the literal source text
grep -qE '^STAGE3_DIGEST=\$\(ws_results_digest "\$RESULTS_ROOT/stage3" "\$STAGE3_SET"' "$CODE_FILE" \
    || fail "run_matrix.sh does not digest the stage-3 results over the expected set"
# shellcheck disable=SC2016  # the single-quoted pattern is the literal source text
seal_line=$(printf '%s\n' "$CODE" | grep -nE '^ws_results_seal "\$RESULTS_ROOT/stage3"' | cut -d: -f1)
first_consume=$(printf '%s\n' "$CODE" | grep -nE '^ *consume_results "' | head -1 | cut -d: -f1)
[ -n "$seal_line" ] && [ -n "$first_consume" ] && [ "$seal_line" -lt "$first_consume" ] \
    || fail "run_matrix.sh does not seal the results BEFORE the first consumer"
# shellcheck disable=SC2016  # the single-quoted pattern is the literal source text
[ "$(sed -n '/^consume_results()/,/^}/p' run_matrix.sh | grep -c 'require_ws_results_intact "\$dir" "\$digest" "\$set"')" -eq 2 ] \
    || fail "consume_results does not re-verify the results both before and after the read"
# shellcheck disable=SC2016  # the single-quoted pattern is the literal source text
sed -n '/^consume_results()/,/^}/p' run_matrix.sh | grep -q 'require_ws_path_identity "\$RESULTS_ROOT" "\$RESULTS_ID"' \
    || fail "consume_results does not re-bind the results root"
# shellcheck disable=SC2016  # the single-quoted patterns are the literal source text
for consumer in 'python3 assert_matrix.py "\$RESULTS_ROOT/stage3"' 'apply_stage3' \
        'cmp "\$RESULTS_ROOT/stage4/det_a.json" "\$RESULTS_ROOT/stage4/det_b.json"' 'judge_mutant' \
        'cmp "\$RESULTS_ROOT/stage6/dry1.txt" "\$RESULTS_ROOT/stage6/dry2.txt"' 'no_diff_proposed'; do
    grep -qE "^ *(if )?consume_results \"[^\"]*\" \"(\\\$RESULTS_ROOT/stage[^\"]*|\\\$rdir)\" \"\\\$[A-Za-z0-9_]*\" \"[^\"]*\" +(.* )?$consumer" "$JOINED_FILE" \
        || fail "consumer '$consumer' does not run through consume_results on the sealed results"
done
# shellcheck disable=SC2016  # the single-quoted pattern is the literal source text
grep -qE '^ *local rdir="\$RESULTS_ROOT/stage5/\$lower"' "$CODE_FILE" \
    || fail "the stage-5 mutant results do not live under the sealed results root"
# shellcheck disable=SC2016  # the single-quoted pattern is the literal source text
grep -qE 'STAGE3_ARGS\+=\("\$RESULTS_ROOT/stage3/\$n"\)' "$CODE_FILE" \
    || fail "the stage-3b apply does not take its inputs from the sealed results"
# shellcheck disable=SC2016  # the single-quoted pattern is the literal source text
grep -q 'doc = json.load(open(os.environ\["MUTANT_RESULT"\]))' "$CODE_FILE" \
    || fail "the stage-5 judge does not read the sealed mutant result"
# shellcheck disable=SC2016  # the single-quoted patterns are the literal source text
if grep -nE 'assert_matrix\.py "\$OUTDIR"|"\$OUTDIR"/\*\.json|> "\$MATRIX_TMP/|"\$MATRIX_TMP/(mut|det_a|det_b|dry1|dry2|after)\.|MATRIX_TMP="\$MATRIX_TMP" python3' "$JOINED_FILE"; then
    fail "run_matrix.sh still writes or reads a result at a mutable operator-visible path"
fi
# shellcheck disable=SC2016  # the single-quoted patterns are the literal source text
if printf '%s\n' "$CODE" | grep -E 'OUTDIR' | grep -vE '^OUTDIR=|^mkdir -p "\$OUTDIR"$|^cp -f "\$RESULTS_ROOT"/stage3/\*\.json "\$OUTDIR"/|^echo '; then
    fail "something other than the export touches \$OUTDIR"
fi
sed -n '/^cleanup_results()/,/^}/p' run_matrix.sh | grep -q 'ws_snapshot_unseal' \
    || fail "cleanup_results does not unseal before deleting (a sealed root would be left behind)"
ws_snapshot_unseal "$R23"
echo "sealed results OK (create-new capture; unprivileged writes refused; a forged result is refused by the digest with nothing applied; extra/missing/unsealed refused; every consumer pinned through consume_results)"

echo "== selftest 24: the read watchdog is a BOUND, not a per-file toll =="
# The hazard this arm pins: the watchdog is
# backgrounded INSIDE ws_file_sha256's command substitution, so without a
# stdout redirect it inherits that pipe; `kill "$w"` reaps the subshell
# while its `sleep` CHILD, holding the same write end, is orphaned and
# lives out the whole bound. The substitution then sees EOF only when the
# bound expires — every regular-file digest cost one full timeout, so a
# tree digest (taken at bind time and re-verified after every prover
# stage) scaled into minutes or tripped the matrix's own timeouts.
# Structural pin first: EVERY watchdog site carries the redirect.
# `grep -c` exits 1 on a zero count, which under `set -e` would kill the
# selftest SILENTLY on the very defect this arm exists to name — the counts
# are data here, not verdicts, so the exit status is deliberately dropped.
# shellcheck disable=SC2016  # the single-quoted pattern is the literal source text
W24_TOTAL=$(grep -c 'kill -TERM "\$me" 2>/dev/null' matrix_lib.sh || true)
W24_REDIR=$(grep -c ') >/dev/null 2>&1 &' matrix_lib.sh || true)
[ "$W24_TOTAL" -ge 3 ] \
    || fail "expected at least 3 watchdog sites in matrix_lib.sh, found $W24_TOTAL"
[ "$W24_REDIR" -eq "$W24_TOTAL" ] \
    || fail "$((W24_TOTAL - W24_REDIR)) watchdog subshell(s) do not redirect stdout — \
one of them inside a command substitution costs a full bound per call"
# Behavioural pin: the structure has the effect. N files under a bound of
# T must cost far LESS than one T; the defect costs N*T. The margin is
# ~26x on a healthy host (measured 0.11s for six files against 31.1s
# without the redirect), so this is a structural separation, not a micro-benchmark.
WS24="$T/ws24"
mkdir -p "$WS24"
for i in 1 2 3 4 5; do printf 'content %s\n' "$i" > "$WS24/f$i.cpp"; done
W24_BOUND=4
W24_START=$SECONDS
D24=$(WS_GIT_OPEN_TIMEOUT=$W24_BOUND ws_tree_digest "$WS24") \
    || fail "digesting five regular files must succeed"
W24_ELAPSED=$((SECONDS - W24_START))
[ -n "$D24" ] || fail "the tree digest produced nothing (apparatus broken)"
[ "$W24_ELAPSED" -lt "$W24_BOUND" ] \
    || fail "five one-line digests took ${W24_ELAPSED}s against a ${W24_BOUND}s bound — \
the watchdog is being paid per file (without the redirect it costs 5x the bound)"
# Anti-vacuity: the bound still FIRES. Deleting the watchdog would satisfy
# the cost pin above and reopen the hang this whole family exists to close.
mkfifo "$WS24/blocked.cpp" # writerless: a plain read would block forever
# shellcheck disable=SC2016  # $1 expands in the child shell
if OUT24=$(timeout 30 env WS_GIT_OPEN_TIMEOUT=2 bash -c \
        '. ./matrix_lib.sh; ws_file_sha256 "$1"' _ "$WS24/blocked.cpp" 2>&1); then
    fail "ws_file_sha256 ACCEPTED a FIFO"
else
    RC24=$?
fi
[ "$RC24" -ne 124 ] || fail "ws_file_sha256 HUNG on the FIFO (killed by timeout)"
case "$OUT24" in
    *"not a regular file"*) ;;
    *) fail "the FIFO was refused for the wrong reason: $OUT24" ;;
esac
# The digest is a function of the BYTES, not of the redirect: same tree,
# same digest, so the redirect changes only WHEN the read completes.
rm "$WS24/blocked.cpp"
D24B=$(WS_GIT_OPEN_TIMEOUT=$W24_BOUND ws_tree_digest "$WS24") || fail "re-digest must succeed"
[ "$D24" = "$D24B" ] || fail "the tree digest is not stable across runs"
echo "watchdog bound OK (every site redirects; five files cost ${W24_ELAPSED}s under a ${W24_BOUND}s bound; the FIFO bound still fires; digest unchanged)"

echo "== selftest 25: the result CAPTURE is create-new and bounded, not just noclobber =="
# This is the write seam that the read-watchdog arm (arm 24)
# does not cover. `noclobber` reads like a create-new guard and is not a bound —
# MEASURED, an existing writerless FIFO at the destination makes `> "$out"`
# BLOCK, because the open waits for a reader before noclobber's check can
# speak, so a planted FIFO would wedge the matrix before the prover runs.
WS25="$T/ws25"
mkdir -p "$WS25"
# Happy path first (anti-vacuity: a seam that refused everything would pass
# every arm below).
ws_capture_result "$WS25/ok.json" echo hello || fail "a fresh destination must capture"
[ "$(cat "$WS25/ok.json")" = hello ] || fail "the captured bytes are not the producer's"
# A producer FAILURE must still be reported as a failure, not swallowed.
if ws_capture_result "$WS25/fails.json" false; then
    fail "a failing producer must not report success"
fi
# Create-new: a pre-placed REGULAR file is refused and left byte-untouched.
printf 'occupant\n' > "$WS25/taken.json"
if OUT25=$(ws_capture_result "$WS25/taken.json" echo forged 2>&1); then
    fail "a pre-placed regular file must refuse the capture"
fi
[ "$(cat "$WS25/taken.json")" = occupant ] || fail "the occupant was overwritten"
case "$OUT25" in
    *"already exists"*) ;;
    *) fail "the occupied capture refused for the wrong reason: $OUT25" ;;
esac
# A SYMLINK at the destination is refused too (it would redirect the write).
ln -s "$WS25/ok.json" "$WS25/linked.json"
if ws_capture_result "$WS25/linked.json" echo forged >/dev/null 2>&1; then
    fail "a symlinked destination must refuse the capture"
fi
[ "$(cat "$WS25/ok.json")" = hello ] || fail "the link target was written through"
# THE PIN: a writerless FIFO at the destination is refused WITHOUT WAITING.
# Without the absence check and the watchdog this blocks forever; `timeout`
# distinguishes a refusal from a hang.
mkfifo "$WS25/blocked.json"
# shellcheck disable=SC2016  # $1 expands in the child shell
if OUT25=$(timeout 30 env WS_GIT_OPEN_TIMEOUT=2 bash -c \
        '. ./matrix_lib.sh; ws_capture_result "$1" echo forged' _ "$WS25/blocked.json" 2>&1); then
    fail "ws_capture_result ACCEPTED a FIFO destination"
else
    RC25=$?
fi
[ "$RC25" -ne 124 ] || fail "ws_capture_result HUNG on a FIFO destination (killed by timeout)"
case "$OUT25" in
    *"already exists"*"FIFO"*) ;;
    *) fail "the FIFO destination refused for the wrong reason: $OUT25" ;;
esac
# THE BOUND IS ON THE OPEN, NOT THE PRODUCER. The producer here is the
# PROVER — clang over a translation unit, legitimately many seconds — so a
# watchdog wrapped around it would kill every real analysis. A producer
# slower than the bound must therefore complete untouched, with its bytes
# intact. (This arm exists because a bound that wraps the producer is the
# easy mistake; nothing else in the suite would catch it, since
# every other producer here returns instantly.)
WS25_SLOW_BOUND=2
# shellcheck disable=SC2016  # $1 expands in the child shell
OUT25=$(timeout 40 env WS_GIT_OPEN_TIMEOUT=$WS25_SLOW_BOUND bash -c \
    '. ./matrix_lib.sh; slow() { sleep 5; echo SLOW_PRODUCER_FINISHED; }; \
     ws_capture_result "$1" slow && cat "$1"' _ "$WS25/slow.json" 2>&1) \
    || fail "a producer slower than the ${WS25_SLOW_BOUND}s open bound was KILLED: $OUT25"
[ "$OUT25" = SLOW_PRODUCER_FINISHED ] \
    || fail "the slow producer's bytes did not survive: $OUT25"
# SCOPE: the absence check is what the arm above exercises. The
# watchdog covers the object planted BETWEEN that check and the redirect,
# which cannot be constructed deterministically without a seam in the
# harness — so it is pinned STRUCTURALLY, the same way arm 24 pins the
# redirect. Reverting the seam to the bare `noclobber` subshell fails here.
# shellcheck disable=SC2016  # the single-quoted pattern is the literal source text
sed -n '/^ws_capture_result()/,/^}/p' matrix_lib.sh | grep -q 'kill -TERM "\$me"' \
    || fail "ws_capture_result no longer carries the watchdog — a destination \
replaced after its absence check can wedge the matrix again"
# shellcheck disable=SC2016  # the single-quoted pattern is the literal source text
sed -n '/^ws_capture_result()/,/^}/p' matrix_lib.sh | grep -qE '\[ -e "\$out" \] \|\| \[ -L "\$out" \]' \
    || fail "ws_capture_result no longer refuses an existing destination"
# shellcheck disable=SC2016  # the single-quoted pattern is the literal source text
sed -n '/^ws_capture_result()/,/^}/p' matrix_lib.sh | grep -qE 'exec \{ofd\}>"\$out"' \
    || fail "ws_capture_result no longer opens the destination on its own \
descriptor — the bound is back around the PROVER, which would kill every real analysis"
echo "capture bound OK (create-new refuses regular/symlink/FIFO destinations without waiting; a producer slower than the bound is NOT killed; the producer's bytes and its failures both survive; the descriptor-scoped bound is pinned structurally)"

echo "== selftest 26: a failed file read REFUSES the tree digest, never truncates it =="
# The bug this arm pins: `ws_tree_digest`'s
# loops are the LAST stage of a pipeline, so bash runs them in their own
# subshell and the `exit 2` inside left only that subshell. Worse, the
# `( ... )` that holds them is itself a NON-FINAL stage of `( ... ) |
# ws_sha256`, so even with `|| exit 2` on each `done` its status is discarded
# and the function returns ws_sha256's — 0. A failed read was therefore
# SWALLOWED: the digest covered a TRUNCATED byte stream and reported success,
# and since a file excluded at bind time stays excluded at every recheck, the
# content-binding gate silently stopped covering it.
WS26="$T/ws26"
mkdir -p "$WS26"
printf 'alpha\n' > "$WS26/a.txt"
# b.txt carries the marker the PATH stub below refuses. Its BYTES are the
# same in the control and in every failing check — only whether the stub is
# on PATH changes — so a refusal there is attributable to the READ failing
# and never to the content having moved (`require_ws_tree_digest` refuses a
# changed digest too, and that arm would otherwise pass for the wrong
# reason).
printf 'UNREADABLE_MARKER\n' > "$WS26/b.txt"
# CONTROL first (anti-vacuity): a healthy tree digests and succeeds.
D26_OK=$(ws_tree_digest "$WS26") || fail "a healthy tree must digest"
[ -n "$D26_OK" ] || fail "the healthy digest is empty"

# About the uid-keyed
# guard below: keying the unreadable-file oracles on the uid cures a false
# FAIL and buys a false PASS. The canonical ros2-bench image carries no `USER`
# directive, so the matrix runs as ROOT — precisely where every assertion in
# that block is skipped, and a `ws_tree_digest` regressed to swallowing a
# failed read would sail through reporting ALL OK. An oracle that disarms
# itself in the one environment the harness actually runs in is not an
# oracle. So the read failure is injected at a seam NO uid can bypass, and
# this half runs unconditionally.
#
# `ws_file_sha256` reads a file as `ws_sha256 < "$1"`, and `ws_sha256`
# resolves `sha256sum` (or `shasum`) through PATH on every call. A stub first
# on PATH that consumes stdin, refuses the one marked file, and hands
# everything else to the real program by ABSOLUTE path is therefore a genuine
# read failure at the exact seam the digest depends on. root ignores
# permission bits; it cannot make a program that exits 2 succeed.
WS26_NAME=sha256sum
WS26_REAL=$(command -v sha256sum 2>/dev/null || true)
if [ -z "$WS26_REAL" ]; then
    WS26_NAME=shasum
    WS26_REAL=$(command -v shasum 2>/dev/null || true)
fi
[ -n "$WS26_REAL" ] || fail "selftest 26 needs sha256sum or shasum on PATH"
WS26_SHIM="$T/shim26"
mkdir -p "$WS26_SHIM"
cat > "$WS26_SHIM/$WS26_NAME" <<SHIM26
#!/bin/sh
# Consume stdin once, then either refuse it or digest it for real. The
# absolute path is baked in at creation so the stub cannot recurse into
# itself through the PATH that puts it first.
t=\$(mktemp) || exit 2
cat > "\$t"
if grep -q UNREADABLE_MARKER "\$t" 2>/dev/null; then
    rm -f "\$t"
    echo "stub: refusing to read the marked file" >&2
    exit 2
fi
"$WS26_REAL" "\$@" < "\$t"
rc=\$?
rm -f "\$t"
exit \$rc
SHIM26
chmod 755 "$WS26_SHIM/$WS26_NAME"
WS26_PATH_SAVE=$PATH
# (assigned, not prefixed: a `VAR=x func` prefix on a FUNCTION call persists
# in the calling shell in bash, which would leave the stub on PATH for every
# later arm.)
PATH="$WS26_SHIM:$PATH"
# ANTI-VACUITY, and it has to come first: over the SAME bytes, the stub must
# produce exactly what the real program produces. Equal digests for one tree
# with the stub on and off is what proves the stub is transparent — so the
# refusal below is the marker being recognised, and not the stub breaking
# every read it touches (which would make every assertion after it pass for
# a reason that has nothing to do with `ws_tree_digest`).
printf 'beta\n' > "$WS26/b.txt"
D26_PLAIN=$(PATH=$WS26_PATH_SAVE; ws_tree_digest "$WS26") \
    || fail "an unmarked tree must digest with the real program"
D26_SHIM=$(ws_tree_digest "$WS26") || fail "the stub must be transparent to an unmarked tree"
[ "$D26_SHIM" = "$D26_PLAIN" ] || fail "the stub altered an unmarked tree's digest \
($D26_SHIM vs $D26_PLAIN) — it is not a transparent delegate, so the refusal it \
produces below would prove nothing"
# Back to the marked bytes for the refusal checks.
printf 'UNREADABLE_MARKER\n' > "$WS26/b.txt"
# Now the marked file: the read fails, and that failure must REACH the caller
# instead of yielding a shorter digest with rc 0.
if D26_BAD=$(ws_tree_digest "$WS26" 2>/dev/null); then
    PATH=$WS26_PATH_SAVE
    fail "ws_tree_digest returned SUCCESS over a tree whose file it could not read \
(digest ${D26_BAD:0:16}) — a truncated digest reported as a good one"
fi
# The gate built on it refuses too. The bytes are unchanged, so with a
# successful read this call would MATCH D26_OK — the refusal is the read.
if require_ws_tree_digest "$WS26" "$D26_OK" "the selftest truncation arm" 2>/dev/null; then
    PATH=$WS26_PATH_SAVE
    fail "require_ws_tree_digest ACCEPTED a tree it could not fully read"
fi
# ...and independently of the CALLER's shell options (see the pipefail note
# in the uid-guarded block below). Driven in a shell that deliberately does
# NOT set pipefail, which is the only place the function's own
# `set -o pipefail` is what answers.
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
if OUT26S=$(timeout 30 bash -c 'set +o pipefail; PATH="$2:$PATH"; . ./matrix_lib.sh; \
ws_tree_digest "$1"' _ "$WS26" "$WS26_SHIM" 2>/dev/null); then
    PATH=$WS26_PATH_SAVE
    fail "ws_tree_digest returned SUCCESS over an unreadable tree when the CALLER had \
not set pipefail (digest ${OUT26S:0:16}) — the refusal depends on the caller's options"
fi
PATH=$WS26_PATH_SAVE
# With the stub off PATH the very same bytes digest again, to the value bound
# before it existed: the stub denied a READ, it did not alter the tree.
D26_AFTER=$(ws_tree_digest "$WS26") || fail "the tree must digest once the read seam is healthy"
[ "$D26_AFTER" = "$D26_OK" ] || fail "the read-failure injection changed the tree's digest"
# The same property once more, through a REAL OS-level denial rather than an
# injected seam — kept because a permission-denied read is the shape the
# function meets in the field, and the stub above can only model it. This
# half is genuinely unobservable as root (root ignores the permission bits,
# so `ws_tree_digest` reads the 0000-mode file happily and these arms would
# FALSE-FAIL on a defect that is only an artifact of the uid), and it is the
# ONLY half that is skipped: the injected-seam block above asserts the very
# same refusal, and it runs on every uid.
if [ "$(id -u)" = 0 ]; then
    echo "  (running as root: chmod 000 does not deny root, so the OS-denial half is not observable — skipped; the PATH-stub half above asserted the same refusal)"
else
    # Make one file unreadable: ws_file_sha256 refuses it, and that refusal
    # must reach the caller instead of yielding a shorter digest with rc 0.
    chmod 000 "$WS26/b.txt"
    if D26_BAD=$(ws_tree_digest "$WS26" 2>/dev/null); then
        chmod 644 "$WS26/b.txt"
        fail "ws_tree_digest returned SUCCESS over a tree it could not fully read \
(digest ${D26_BAD:0:16}) — a truncated digest reported as a good one"
    fi
    chmod 644 "$WS26/b.txt"
    # The gate built on it refuses too, rather than passing a stale digest.
    chmod 000 "$WS26/b.txt"
    if require_ws_tree_digest "$WS26" "$D26_OK" "the selftest truncation arm" 2>/dev/null; then
        chmod 644 "$WS26/b.txt"
        fail "require_ws_tree_digest ACCEPTED a tree it could not fully read"
    fi
    chmod 644 "$WS26/b.txt"
    # ...and independently of the CALLER's shell options. This selftest and
    # run_matrix.sh both set `-o pipefail`, which subshells inherit, so the
    # `|| exit 2` guards alone would satisfy every check above — but the
    # function is a sourced library, and a caller without pipefail would
    # otherwise read ws_sha256's 0 over a truncated stream. Driven in a shell
    # that deliberately does NOT set pipefail, which is the only place the
    # function's own `set -o pipefail` is what answers.
    chmod 000 "$WS26/b.txt"
    # shellcheck disable=SC2016  # $1 expands in the child shell
    if OUT26=$(timeout 30 bash -c 'set +o pipefail; . ./matrix_lib.sh; ws_tree_digest "$1"' \
            _ "$WS26" 2>/dev/null); then
        chmod 644 "$WS26/b.txt"
        fail "ws_tree_digest returned SUCCESS over an unreadable tree when the CALLER \
had not set pipefail (digest ${OUT26:0:16}) — the refusal depends on the caller's options"
    fi
    chmod 644 "$WS26/b.txt"
fi
# Outside the guard: the digest is a function of the BYTES, so a healthy tree
# digests identically whatever the uid.
D26_BACK=$(ws_tree_digest "$WS26") || fail "the restored tree must digest again"
[ "$D26_BACK" = "$D26_OK" ] || fail "the restored tree's digest changed"
echo "tree digest refusal OK (a failed read refuses instead of truncating — asserted on EVERY uid via a PATH read-failure seam, and again via a real OS denial when not root; healthy tree unaffected; the gate refuses too; refusal holds without caller pipefail)"

echo "== selftest 27: mutant prover binaries never outlive the matrix =="
# Stage 5 deletes each mutant prover binary
# after judging it, but that deletion sat on the paths THROUGH the
# judgement — an abort mid-mutant (a `set -e` failure, an `exit 2` from a
# bound-input re-check, a Ctrl-C) left `…-mutant-<name>` in the tool
# directory, which is the repo checkout bind-mounted into the container. A
# prover with a proof compiled OUT then outlives the container, sitting
# where a later run (or an operator) reaches for "the tool".
#
# The child below models run_matrix.sh's wiring EXACTLY — the library
# restore in ONE EXIT trap, which is the whole of it — and is driven to each
# ending. There is no signal handler to model: measurement showed bash runs
# the EXIT trap on a fatal signal anyway, and INT/TERM/HUP handlers would
# also suppress `cleanup_ws` by re-raising after `trap - EXIT`. Sub-arm (c)
# is what pins that measurement. The structural pins at the end tie the model to
# production, including that production installs exactly ONE trap.
WS27_TOOL="$T/tools27"
mkdir -p "$WS27_TOOL"
WS27_PROD="$WS27_TOOL/cerulion-ros2-migrate-clang"
# A STAND-IN name, deliberately not any real mutant's: this arm models the
# restore over a scratch tool directory and never calls build.sh, so the
# suffix only has to match ws_mutant_stale_list's `…-mutant-*` glob. Naming a
# real mutant here would tie this file to whichever mutants run_matrix.sh
# defines, and read as though the arm builds it.
WS27_MUT="$WS27_TOOL/cerulion-ros2-migrate-clang-mutant-selftest_model"
printf 'PRODUCTION PROVER\n' > "$WS27_PROD"
chmod 755 "$WS27_PROD"
WS27_PROD_SHA=$(ws_file_sha256 "$WS27_PROD") || fail "the stand-in prover must digest"
cat > "$T/child27.sh" <<'CHILD27'
#!/usr/bin/env bash
# $1 matrix_lib.sh, $2 tool dir, $3 ending, $4 handshake dir.
set -euo pipefail
# shellcheck source=/dev/null
. "$1"
TOOLDIR=$2
MUT="$TOOLDIR/cerulion-ros2-migrate-clang-mutant-selftest_model"
if [ "$3" = notrap ]; then
    # The NO-RESTORE control: the same abort with no restore wired at all.
    printf 'MUTANT PROVER\n' > "$MUT"
    chmod 755 "$MUT"
    printf '%s\n' "$MUT" > "$4/created"
    exit 2
fi
ws_mutants_bind "$TOOLDIR" "$TOOLDIR/cerulion-ros2-migrate-clang" || exit 2
OUT27=$4  # a function's $4 is its OWN argument, not the script's
# Production's trap calls the restore immediately after its `code=$?`
# capture — second of five, and it must stay second: moving it above the
# capture would make `$?` read the restore's status and silently break the
# workspace-cleanup gate that keys on it. The marker below proves the trap
# kept going in the state that matters (bound, with work to do), which the
# unbound (a2) model cannot reach.
cleanup() { ws_mutants_restore; printf 'after\n' > "$OUT27/after"; }
trap cleanup EXIT
ws_mutant_register "$MUT"
printf 'MUTANT PROVER\n' > "$MUT"
chmod 755 "$MUT"
# Bound after the "build", as production binds after build.sh —
# an entry with no identity is refused by the sweep, not removed, so the
# abort endings below model an abort AFTER the bind (the shape stage 5
# spends its minutes in).
ws_mutant_bind_identity "$MUT" || { printf 'bind-failed\n' > "$OUT27/bind-failed"; exit 3; }
[ -f "$MUT" ] || exit 3
printf '%s\n' "$MUT" > "$4/created"
case $3 in
    fail)
        exit 2  # an abort while the mutant binary is on disk
        ;;
    tamper)
        printf 'REPLACED PROVER\n' > "$TOOLDIR/cerulion-ros2-migrate-clang"
        ws_file_sha256 "$TOOLDIR/cerulion-ros2-migrate-clang" > "$4/tampered"
        exit 0
        ;;
    park)
        exec {gfd}<>"$4/go"
        printf 'ready\n' > "$4/ready"
        for _ in $(seq 1 300); do
            if read -r -t 1 -u "$gfd" _; then break; fi
        done
        ;;
    *)
        exit 4
        ;;
esac
CHILD27
chmod 755 "$T/child27.sh"

# (a) NO-RESTORE CONTROL, first: without the restore the abort really does
# leave the binary behind. Without this the "no mutant remains" oracles
# below would pass against a harness that simply never created one.
W27A="$T/w27a"
mkdir "$W27A"
if timeout 60 bash "$T/child27.sh" "$PWD/matrix_lib.sh" "$WS27_TOOL" notrap "$W27A"; then
    fail "the modelled abort must exit non-zero (apparatus broken)"
fi
[ -s "$W27A/created" ] || fail "the control child did not create a mutant binary (apparatus broken)"
[ -f "$WS27_MUT" ] \
    || fail "control: an abort with no restore wired must LEAVE the mutant binary behind — \
if it does not, this arm's oracles prove nothing"
rm -f "$WS27_MUT"

# (a2) the restore must not abort the trap it runs in, checked BEFORE the
# behavioural arms below: a regression that makes ws_mutants_restore return
# non-zero surfaces in (d) as "the tamper child must exit 0 (apparatus
# broken)", which sends the reader hunting for a harness bug. Run first, it
# names the real defect — the exit status a trap must not rewrite.
# `set -e`: a non-zero return there aborts the REST of that trap — the
# snapshot, the results root and the workspace cleanup — and, as a trap's
# last command, rewrites the script's exit status. Every path through the
# function ends 0 today, so this is not pinning the `return 0` line; it is
# pinning the PROPERTY, which a later edit ending the function with a bare
# test would break (that mutation fails this arm). Driven the way the
# function is really called.
cat > "$T/trap27.sh" <<'TRAP27'
#!/usr/bin/env bash
set -euo pipefail
# shellcheck source=/dev/null
. "$1"
OUT=$2  # a function's $2 is its OWN argument, not the script's
cleanup() { ws_mutants_restore; printf 'reached\n' > "$OUT/after"; }
trap cleanup EXIT
exit 7
TRAP27
chmod 755 "$T/trap27.sh"
W27A2="$T/w27a2"
mkdir "$W27A2"
RC27A2=0
timeout 60 bash "$T/trap27.sh" "$PWD/matrix_lib.sh" "$W27A2" || RC27A2=$?
[ "$RC27A2" -eq 7 ] \
    || fail "the trap model exited $RC27A2, not 7 — the exit status a trap must not rewrite"
[ -f "$W27A2/after" ] \
    || fail "ws_mutants_restore with nothing bound ABORTED the rest of an EXIT trap under \
set -e — in run_matrix.sh that is the snapshot, the results root and the workspace left behind"
if ! ( set -e; . ./matrix_lib.sh; ws_mutants_restore ); then
    fail "ws_mutants_restore must succeed when there is nothing to restore"
fi

# (b) the abort path: a non-zero exit mid-mutant, restore wired.
W27B="$T/w27b"
mkdir "$W27B"
if timeout 60 bash "$T/child27.sh" "$PWD/matrix_lib.sh" "$WS27_TOOL" fail "$W27B" \
        2> "$W27B/err"; then
    fail "the modelled abort must exit non-zero (apparatus broken)"
fi
[ ! -e "$W27B/bind-failed" ] \
    || fail "the model child could not BIND its own freshly built mutant — the bind refuses \
production's own artifact (the stamp, its read-back, or the key's shape is broken): \
$(cat "$W27B/err" 2>/dev/null)"
[ -s "$W27B/created" ] || fail "the child did not create a mutant binary (apparatus broken)"
[ ! -e "$WS27_MUT" ] \
    || fail "an aborted mutant run LEFT $WS27_MUT on disk — a prover with a proof compiled \
out outlives the matrix that built it"
[ -f "$W27B/after" ] \
    || fail "the restore aborted the rest of the EXIT trap in the BOUND state — in
run_matrix.sh that is the snapshot, the results root and the workspace left behind"
grep -q "removed 1 leftover mutant prover" "$W27B/err" \
    || fail "the restore removed the binary SILENTLY — that breadcrumb is the only thing that \
tells an operator their tool directory was touched: $(cat "$W27B/err")"
[ "$(ws_file_sha256 "$WS27_PROD")" = "$WS27_PROD_SHA" ] \
    || fail "the production prover changed across an aborted run"

# (c) the SIGNAL path, which is the likeliest abort of all (a Ctrl-C during
# the minutes stage 5 takes) and is a DIFFERENT path through bash than (b):
# the shell is terminated by the signal rather than exiting. run_matrix.sh
# covers it with the EXIT trap alone, which rests on bash running that trap
# when a fatal signal terminates the shell — MEASURED here for SIGTERM (and
# by hand for SIGINT/SIGHUP) rather than assumed, because common folklore
# says the OPPOSITE and would justify a signal handler.
# On a shell where it stops holding, this arm fails instead of going quiet,
# and the startup refusal is what keeps the leftover loud meanwhile.
W27C="$T/w27c"
mkdir "$W27C"
mkfifo "$W27C/ready" "$W27C/go"
bash "$T/child27.sh" "$PWD/matrix_lib.sh" "$WS27_TOOL" park "$W27C" &
C27=$!
exec {r27}<>"$W27C/ready"
read -r -t 60 -u "$r27" _ || fail "the parked child never reached its park seam"
exec {r27}<&-
[ ! -e "$W27C/bind-failed" ] \
    || fail "the model child could not BIND its own freshly built mutant on the signal path"
[ -f "$WS27_MUT" ] || fail "the parked child must hold a mutant binary on disk (apparatus broken)"
kill -TERM "$C27"
RC27=0
wait "$C27" || RC27=$?
[ "$RC27" -eq 143 ] \
    || fail "the signalled child exited $RC27, not 143 — it did not die of the re-raised \
SIGTERM, so this arm is not measuring the signal path"
[ ! -e "$WS27_MUT" ] \
    || fail "a SIGTERM'd mutant run LEFT $WS27_MUT on disk — this shell does not run the EXIT \
trap when a signal terminates it, which is the assumption run_matrix.sh's single-trap coverage \
rests on; it needs a signal handler here"
[ "$(ws_file_sha256 "$WS27_PROD")" = "$WS27_PROD_SHA" ] \
    || fail "the production prover changed across a signalled run"

# (d) the pristine half: whatever replaced the production prover during the
# run, the binary a LATER run reaches for is the one this run built.
W27D="$T/w27d"
mkdir "$W27D"
timeout 60 bash "$T/child27.sh" "$PWD/matrix_lib.sh" "$WS27_TOOL" tamper "$W27D" \
        2> "$W27D/err" \
    || fail "the tamper child must exit 0 (apparatus broken; if the restore's own exit status \
leaked into the trap, sub-arm (a2) names that defect): $(cat "$W27D/err" 2>/dev/null)"
[ ! -e "$W27D/bind-failed" ] \
    || fail "the model child could not BIND its own freshly built mutant — the bind refuses \
production's own artifact (the stamp, its read-back, or the key's shape is broken): \
$(cat "$W27D/err" 2>/dev/null)"
[ -s "$W27D/tampered" ] || fail "the tamper child recorded no replacement digest (apparatus broken)"
[ "$(cat "$W27D/tampered")" != "$WS27_PROD_SHA" ] \
    || fail "the tamper wrote the SAME bytes — the restore below would pass vacuously"
[ "$(ws_file_sha256 "$WS27_PROD")" = "$WS27_PROD_SHA" ] \
    || fail "a production prover replaced during the run was NOT restored from the pristine copy"
[ ! -e "$WS27_MUT" ] || fail "the tamper run left a mutant binary behind"
grep -q "is not the binary this run built" "$W27D/err" \
    || fail "the production prover was restored SILENTLY — the operator is never told their \
tool directory was touched: $(cat "$W27D/err")"
[ -x "$WS27_PROD" ] \
    || fail "the restored production prover is not EXECUTABLE — byte-correct and unusable is \
the shape that passes a digest check and fails at the next run"
[ -f "$W27D/after" ] || fail "the restore aborted the rest of the EXIT trap on the tamper path"

# (e) a mutant binary already on disk is REFUSED, never deleted: this
# harness removes no path it did not create, and a leftover is exactly the
# artifact that makes the next run's verdict untrustworthy.
[ "$(ws_mutant_stale_list "$WS27_TOOL")" = "" ] \
    || fail "the tool directory should be clean before the refusal check"
ws_mutants_require_clean "$WS27_TOOL" \
    || fail "a clean tool directory must be ACCEPTED (anti-vacuity for the refusal below)"
printf 'STALE MUTANT\n' > "$WS27_MUT"
if OUT27=$(ws_mutants_require_clean "$WS27_TOOL" 2>&1); then
    rm -f "$WS27_MUT"
    fail "a tool directory already holding a mutant prover binary must be REFUSED"
fi
case "$OUT27" in
    *"$WS27_MUT"*) ;;
    *) fail "the refusal does not name the leftover binary: $OUT27" ;;
esac
[ -f "$WS27_MUT" ] || fail "the refusal DELETED a binary this run did not create"
rm -f "$WS27_MUT"

# (h) the guards inside the module itself. Each is here because
# an unpinned guard is an inert guard, and a guard can be inert at its
# production call site without any test noticing.
W27H="$T/w27h"
mkdir -p "$W27H/tools"
printf 'PROD\n' > "$W27H/tools/cerulion-ros2-migrate-clang"
chmod 755 "$W27H/tools/cerulion-ros2-migrate-clang"
# (h1) register without a bind REFUSES, rather than half-working (the removal
# half would still run while the pristine half silently would not exist).
# shellcheck disable=SC2016  # $1 expands in the child shell
if OUT27H=$(timeout 30 bash -c '. "$1"; ws_mutant_register "$2/x"' \
        _ "$PWD/matrix_lib.sh" "$W27H/tools" 2>&1); then
    fail "ws_mutant_register ACCEPTED a registration with no production prover bound"
fi
case "$OUT27H" in
    *"no production prover bound"*) ;;
    *) fail "the register refusal does not name its cause: $OUT27H" ;;
esac
# (h2) a SECOND bind refuses: it would leak the first pristine copy and clear
# the registry, orphaning every mutant registered against the first bind.
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
# The child restores before exiting: a successful first bind makes a pristine
# copy under TMPDIR, which lives OUTSIDE the selftest's own temp root and so
# survives its cleanup trap. Measured on a draft: one stranded copy per run.
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
if OUT27H2=$(timeout 30 bash -c '. "$1"
ws_mutants_bind "$2" "$2/cerulion-ros2-migrate-clang" || exit 9
ws_mutants_bind "$2" "$2/cerulion-ros2-migrate-clang"
rc=$?
ws_mutants_restore >/dev/null 2>&1
exit $rc' _ "$PWD/matrix_lib.sh" "$W27H/tools" 2>&1); then
    fail "ws_mutants_bind ACCEPTED a rebind over a live binding"
fi
case "$OUT27H2" in
    *"already bound"*) ;;
    *) fail "the rebind refusal does not name its cause: $OUT27H2" ;;
esac
# (h3) a mutant binary this run did NOT register survives the restore — and
# must be REPORTED rather than pass in silence with rc 0. This is the half the
# registry cannot see, and the one a dropped register status walks straight
# into.
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
OUT27H3=$(timeout 30 bash -c '. "$1"; ws_mutants_bind "$2" "$2/cerulion-ros2-migrate-clang" \
|| exit 9; printf X > "$2/cerulion-ros2-migrate-clang-mutant-unregistered"; \
ws_mutants_restore' _ "$PWD/matrix_lib.sh" "$W27H/tools" 2>&1) \
    || fail "the restore must still succeed while reporting an unregistered leftover"
case "$OUT27H3" in
    *"survived this run and were not registered by it"*) ;;
    *) fail "an UNREGISTERED mutant binary survived the restore in SILENCE — the module's \
promise read backwards: $OUT27H3" ;;
esac
[ -f "$W27H/tools/cerulion-ros2-migrate-clang-mutant-unregistered" ] \
    || fail "the sweep DELETED a binary this run did not create — report, never remove"
rm -f "$W27H/tools/cerulion-ros2-migrate-clang-mutant-unregistered"
# (h4) a registered path with NO bound identity — a DANGLING SYMLINK, and a
# real file whose build never completed — is REFUSED by the sweep: named,
# left in place, latched as a restore failure (no identity ⇒ no
# removal — before it, both were swept blind, the destructive class the
# build-bind gap made concrete). `-e` follows symlinks, so an existence test
# alone would skip the link and neither name nor count it: the sweep must
# SEE it to refuse it. The anti-vacuity half is the same child registering
# AND binding a real file in the same run — that one must be removed, or the
# loop is refusing everything.
W27H_LINK="$W27H/tools/cerulion-ros2-migrate-clang-mutant-dangling"
W27H_REAL="$W27H/tools/cerulion-ros2-migrate-clang-mutant-real"
W27H_BOUND="$W27H/tools/cerulion-ros2-migrate-clang-mutant-bound"
# Every step carries `|| exit 9` and the LINK'S EXISTENCE IS ASSERTED IN THE
# CHILD, so a child whose `ln` failed cannot satisfy the oracle having
# created nothing (measured: without the child-side assert, a stub
# `ln` that exits 1 lets the arm PASS). Stderr is captured with stdout, so a
# setup refusal names itself instead of surfacing as the restore's failure.
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
OUT27H4=$(timeout 30 bash -c '. "$1" || exit 9
ws_mutants_bind "$2" "$2/cerulion-ros2-migrate-clang" || exit 9
ln -s "$2/nonexistent-target" "$2/cerulion-ros2-migrate-clang-mutant-dangling" || exit 9
[ -L "$2/cerulion-ros2-migrate-clang-mutant-dangling" ] || exit 9
[ -e "$2/cerulion-ros2-migrate-clang-mutant-dangling" ] && exit 9   # must DANGLE
printf X > "$2/cerulion-ros2-migrate-clang-mutant-real" || exit 9
printf X > "$2/cerulion-ros2-migrate-clang-mutant-bound" || exit 9
ws_mutant_register "$2/cerulion-ros2-migrate-clang-mutant-dangling" || exit 9
ws_mutant_register "$2/cerulion-ros2-migrate-clang-mutant-real" || exit 9
ws_mutant_register "$2/cerulion-ros2-migrate-clang-mutant-bound" || exit 9
ws_mutant_bind_identity "$2/cerulion-ros2-migrate-clang-mutant-bound" || exit 9
ws_mutants_restore
if ws_mutants_restore_failed; then echo LATCHED; else echo CLEAN; fi' \
    _ "$PWD/matrix_lib.sh" "$W27H/tools" 2>&1) \
    || fail "the never-bound-entries child failed before its oracle: $OUT27H4"
[ ! -e "$W27H_BOUND" ] \
    || fail "the restore left a registered AND BOUND mutant binary behind (apparatus broken — \
the never-bound assertions below would prove nothing)"
[ -L "$W27H_LINK" ] \
    || fail "a registered DANGLING SYMLINK with no identity was REMOVED by the sweep — no \
identity ⇒ no removal; the sweep cannot prove whose link that is"
[ -f "$W27H_REAL" ] \
    || fail "a registered file with no identity (a build that never completed) was REMOVED by \
the sweep — a blind sweep is the destructive class this arm forbids"
case "$OUT27H4" in
    *"$W27H_LINK"*"never bound to an identity"*|*"never bound to an identity"*"$W27H_LINK"*) ;;
    *) fail "the sweep did not NAME the dangling link it refused — \`[ -e ]\` follows symlinks, \
so an existence test alone skips exactly what it must see: $OUT27H4" ;;
esac
case "$OUT27H4" in
    *"$W27H_REAL"*) ;;
    *) fail "the sweep did not NAME the never-bound file it refused: $OUT27H4" ;;
esac
case "$OUT27H4" in
    *LATCHED*) ;;
    *) fail "registered entries with no identity were left behind WITHOUT failing the run: \
$OUT27H4" ;;
esac
rm -f "$W27H_LINK" "$W27H_REAL"

# (h5) the prover DELETED during the run — the only path where `cp` creates
# the destination from the 0400 pristine copy, so the mode must be put back.
# Sub-arm (d) cannot see this: it tampers by truncating, which preserves 755,
# so `cp` preserves it and the chmod is a no-op there. Both the chmod and the
# executability check survived mutation until this arm existed.
W27H5="$T/w27h5"
mkdir -p "$W27H5/tools"
printf 'PROD\n' > "$W27H5/tools/cerulion-ros2-migrate-clang"
chmod 755 "$W27H5/tools/cerulion-ros2-migrate-clang"
W27H5_SHA=$(ws_file_sha256 "$W27H5/tools/cerulion-ros2-migrate-clang")
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
timeout 30 bash -c '. "$1" || exit 9
ws_mutants_bind "$2" "$2/cerulion-ros2-migrate-clang" || exit 9
rm -f "$2/cerulion-ros2-migrate-clang" || exit 9
ws_mutants_restore' _ "$PWD/matrix_lib.sh" "$W27H5/tools" 2> "$W27H5/err" \
    || fail "the restore must succeed after the prover was deleted: $(cat "$W27H5/err")"
[ "$(ws_file_sha256 "$W27H5/tools/cerulion-ros2-migrate-clang")" = "$W27H5_SHA" ] \
    || fail "a prover DELETED during the run was not restored byte-for-byte"
[ -x "$W27H5/tools/cerulion-ros2-migrate-clang" ] \
    || fail "the prover was restored from the 0400 pristine copy and left NOT EXECUTABLE — \
byte-correct and unusable, which a digest check cannot see"
[ "$(ws_object_mode "$W27H5/tools/cerulion-ros2-migrate-clang")" = "755" ] \
    || fail "the restored prover's mode is not the one it was bound with"

# (h6) a MODE-ONLY break: the bytes never change, so the byte compare
# short-circuits, and a restore keyed on bytes alone leaves the prover unusable in silence.
# The break happens INSIDE the child, AFTER the bind — breaking it first
# would make the bind capture the broken mode as the one to restore, and the
# arm would be asserting the wrong thing.
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
timeout 30 bash -c '. "$1" || exit 9
ws_mutants_bind "$2" "$2/cerulion-ros2-migrate-clang" || exit 9
chmod 0644 "$2/cerulion-ros2-migrate-clang" || exit 9
ws_mutants_restore' _ "$PWD/matrix_lib.sh" "$W27H5/tools" 2> "$W27H5/err6" \
    || fail "the restore must succeed over a mode-only break: $(cat "$W27H5/err6")"
[ -x "$W27H5/tools/cerulion-ros2-migrate-clang" ] \
    || fail "a prover whose MODE was broken (bytes untouched) was left unusable — the byte \
compare short-circuits, so the mode check must sit outside it"

# (h7) the restore must not be able to abort the trap THROUGH ITS OWN
# REPORTING. With stderr closed, a failing `echo … >&2` under `set -e` aborts
# before `return 0` — skipping the rest of run_matrix.sh's EXIT trap and
# rewriting the script's exit status. Driven in the BOUND state with work to
# do, which is the state (a2) cannot reach.
W27H7="$T/w27h7"
mkdir -p "$W27H7/tools"
printf 'PROD\n' > "$W27H7/tools/cerulion-ros2-migrate-clang"
chmod 755 "$W27H7/tools/cerulion-ros2-migrate-clang"
cat > "$T/trap27b.sh" <<'TRAP27B'
#!/usr/bin/env bash
set -euo pipefail
# shellcheck source=/dev/null
. "$1"
OUT=$2
cleanup() { ws_mutants_restore 2>&-; printf 'reached
' > "$OUT/after"; }
trap cleanup EXIT
ws_mutants_bind "$OUT/tools" "$OUT/tools/cerulion-ros2-migrate-clang" || exit 9
ws_mutant_register "$OUT/tools/cerulion-ros2-migrate-clang-mutant-x" || exit 9
printf X > "$OUT/tools/cerulion-ros2-migrate-clang-mutant-x"
# Bound, as production binds after its build — an unbound entry
# is refused by the restore, and this arm is about the restore's REPORTING.
ws_mutant_bind_identity "$OUT/tools/cerulion-ros2-migrate-clang-mutant-x" || exit 9
exit 7
TRAP27B
chmod 755 "$T/trap27b.sh"
RC27H7=0
timeout 60 bash "$T/trap27b.sh" "$PWD/matrix_lib.sh" "$W27H7" || RC27H7=$?
[ "$RC27H7" -eq 7 ] \
    || fail "the trap model exited $RC27H7, not 7 — the restore's own REPORTING aborted the \
trap and rewrote the exit status with stderr closed"
[ -f "$W27H7/after" ] \
    || fail "the restore aborted the rest of the EXIT trap when its reporting could not write \
— in run_matrix.sh that is the snapshot, the results root and the workspace left behind"
[ ! -e "$W27H7/tools/cerulion-ros2-migrate-clang-mutant-x" ] \
    || fail "the restore did not remove the mutant when its reporting could not write"

# (h8) a DANGLING symlink named like a mutant must be seen by the STARTUP
# refusal too. `ws_mutant_stale_list` feeds both that gate and the end-of-run
# sweep, and it is the module's only backstop for a SIGKILL or a hand-built
# mutant — a blind spot there is a blind spot in the backstop. The
# registered-removal loop got this guard first; its sibling survived a
# mutation sweep without it until this arm existed.
W27H8="$T/w27h8"
mkdir -p "$W27H8/tools"
ln -s "$W27H8/nonexistent-target" \
    "$W27H8/tools/cerulion-ros2-migrate-clang-mutant-dangling"
[ -L "$W27H8/tools/cerulion-ros2-migrate-clang-mutant-dangling" ] \
    && [ ! -e "$W27H8/tools/cerulion-ros2-migrate-clang-mutant-dangling" ] \
    || fail "the arm needs a link that exists and DANGLES (apparatus broken)"
if OUT27H8=$(ws_mutants_require_clean "$W27H8/tools" 2>&1); then
    fail "a tool directory holding a DANGLING mutant symlink was ACCEPTED — \`[ -e ]\` \
follows symlinks, so the startup gate cannot see one"
fi
case "$OUT27H8" in
    *"already present"*) ;;
    *) fail "the dangling-symlink refusal does not name its cause: $OUT27H8" ;;
esac

# (h9) the executability report is the LAST backstop, and it has to be
# reachable on its own: the mode restore above normally makes the prover
# executable, so this reporter only speaks when the mode it was BOUND with is
# itself non-executable. Bound at 0644, restored to 0644 — correct, and
# useless to run, which is exactly what the operator needs told.
W27H9="$T/w27h9"
mkdir -p "$W27H9/tools"
printf 'PROD\n' > "$W27H9/tools/cerulion-ros2-migrate-clang"
chmod 0644 "$W27H9/tools/cerulion-ros2-migrate-clang"
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
OUT27H9=$(timeout 30 bash -c '. "$1" || exit 9
ws_mutants_bind "$2" "$2/cerulion-ros2-migrate-clang" || exit 9
ws_mutants_restore' _ "$PWD/matrix_lib.sh" "$W27H9/tools" 2>&1) \
    || fail "the restore must succeed over a non-executable prover: $OUT27H9"
case "$OUT27H9" in
    *"not executable"*) ;;
    *) fail "a prover that is not executable was reported as nothing at all: $OUT27H9" ;;
esac

# (h10) a SYMLINK planted at the prover pathname before cleanup must not be
# followed. The alternative is reproducible: `cmp`/`cp`/`chmod` all follow
# it, so a restore by pathname rewrites an UNRELATED file's bytes and sets its mode to the
# prover's while the pathname stays a symlink. The bind-time regular-file
# check cannot help — it was true minutes earlier. The oracle is the TARGET:
# its bytes and its mode must both survive untouched, and the pathname must be
# a regular file again afterwards.
W27H10="$T/w27h10"
mkdir -p "$W27H10/tools"
printf 'PROD\n' > "$W27H10/tools/cerulion-ros2-migrate-clang"
chmod 755 "$W27H10/tools/cerulion-ros2-migrate-clang"
printf 'AN UNRELATED FILE\n' > "$W27H10/victim"
chmod 0600 "$W27H10/victim"
W27H10_VICTIM_SHA=$(ws_file_sha256 "$W27H10/victim")
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
timeout 30 bash -c '. "$1" || exit 9
ws_mutants_bind "$2/tools" "$2/tools/cerulion-ros2-migrate-clang" || exit 9
rm -f "$2/tools/cerulion-ros2-migrate-clang" || exit 9
ln -s "$2/victim" "$2/tools/cerulion-ros2-migrate-clang" || exit 9
[ -L "$2/tools/cerulion-ros2-migrate-clang" ] || exit 9
ws_mutants_restore' _ "$PWD/matrix_lib.sh" "$W27H10" 2> "$W27H10/err" \
    || fail "the restore must succeed over a planted symlink: $(cat "$W27H10/err")"
[ "$(ws_file_sha256 "$W27H10/victim")" = "$W27H10_VICTIM_SHA" ] \
    || fail "the restore FOLLOWED a symlink planted at the prover pathname and overwrote an \
unrelated file's bytes"
[ "$(ws_object_mode "$W27H10/victim")" = "600" ] \
    || fail "the restore followed the symlink and changed an unrelated file's MODE to the \
prover's ($(ws_object_mode "$W27H10/victim"))"
[ -f "$W27H10/tools/cerulion-ros2-migrate-clang" ] \
    && [ ! -L "$W27H10/tools/cerulion-ros2-migrate-clang" ] \
    || fail "the prover pathname is still a symlink after the restore — the rename should \
have replaced the NAME"
[ -x "$W27H10/tools/cerulion-ros2-migrate-clang" ] \
    || fail "the prover replaced by rename is not executable"

# (h12) the destination is re-validated before it is READ, not only before it
# is written. The atomic rename already protects a symlink's TARGET (h10), so
# what the `-L`/`-f` test adds is refusing to read through the pathname at
# all — and the shape that proves it is a writerless FIFO: `cmp` against one
# BLOCKS FOREVER, hanging the EXIT trap of every run that meets it. Bounded,
# so a hang fails the arm instead of hanging the selftest.
W27H12="$T/w27h12"
mkdir -p "$W27H12/tools"
printf 'PROD\n' > "$W27H12/tools/cerulion-ros2-migrate-clang"
chmod 755 "$W27H12/tools/cerulion-ros2-migrate-clang"
RC27H12=0
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
timeout 30 bash -c '. "$1" || exit 9
ws_mutants_bind "$2/tools" "$2/tools/cerulion-ros2-migrate-clang" || exit 9
rm -f "$2/tools/cerulion-ros2-migrate-clang" || exit 9
mkfifo "$2/tools/cerulion-ros2-migrate-clang" || exit 9
[ -p "$2/tools/cerulion-ros2-migrate-clang" ] || exit 9
ws_mutants_restore' _ "$PWD/matrix_lib.sh" "$W27H12" 2> "$W27H12/err" || RC27H12=$?
[ "$RC27H12" -ne 124 ] \
    || fail "the restore HUNG on a writerless FIFO planted at the prover pathname — it read \
through the destination instead of re-validating it first, and that hang is in an EXIT trap"
[ "$RC27H12" -eq 0 ] \
    || fail "the restore over a FIFO exited $RC27H12: $(cat "$W27H12/err")"
[ -f "$W27H12/tools/cerulion-ros2-migrate-clang" ] \
    && [ ! -p "$W27H12/tools/cerulion-ros2-migrate-clang" ] \
    || fail "the prover pathname is still a FIFO after the restore"

# (h11) a restore that CANNOT put the tool directory back must fail the run.
# Reporting it and returning 0 leaves an otherwise-green matrix exiting 0 with
# a corrupted prover, which is "a mutant outlives the matrix" by another
# route. The injection removes the PRISTINE COPY so the copy-back fails —
# uid-independent on purpose: a chmod-based one is inert as root, and the
# container runs as root (the ORACLES-vs-ROOT rule).
cat > "$T/trap27c.sh" <<'TRAP27C'
#!/usr/bin/env bash
set -euo pipefail
# shellcheck source=/dev/null
. "$1"
OUT=$2
cleanup() {
    ws_mutants_restore
    printf 'reached\n' > "$OUT/after"
    if ws_mutants_restore_failed; then
        exit 2
    fi
}
trap cleanup EXIT
ws_mutants_bind "$OUT/tools" "$OUT/tools/cerulion-ros2-migrate-clang" || exit 9
printf 'CORRUPTED\n' > "$OUT/tools/cerulion-ros2-migrate-clang"
rm -f "$WS_MUTANT_PRISTINE"          # the copy-back can no longer succeed
exit 0
TRAP27C
chmod 755 "$T/trap27c.sh"
W27H11="$T/w27h11"
mkdir -p "$W27H11/tools"
printf 'PROD\n' > "$W27H11/tools/cerulion-ros2-migrate-clang"
chmod 755 "$W27H11/tools/cerulion-ros2-migrate-clang"
RC27H11=0
timeout 60 bash "$T/trap27c.sh" "$PWD/matrix_lib.sh" "$W27H11" > /dev/null 2> "$W27H11/err" \
    || RC27H11=$?
[ "$RC27H11" -ne 0 ] \
    || fail "a run whose restore FAILED exited 0 — an otherwise-green matrix would report \
success with a corrupted production prover: $(cat "$W27H11/err")"
[ -f "$W27H11/after" ] \
    || fail "promoting the restore failure skipped the rest of the trap — the failure must be \
LATCHED and consulted last, never returned"
grep -q "could NOT be restored" "$W27H11/err" \
    || fail "the failing restore did not say so: $(cat "$W27H11/err")"

# (h13) a run whose BODY SUCCEEDED but whose restore FAILED must retain its
# scratch workspace. The promotion that turns a failed restore into exit 2
# has to run LAST (it is the only thing in the trap that can `exit`, and
# moving it earlier skips the remaining cleanups — sub-arm (a2)'s defect),
# so by the time it fires cleanup_ws_workspace has already seen `code == 0`
# and deleted the very tree an operator would read to find out what the
# matrix left behind. The failing restore is what makes this reachable:
# without it, a body that exited 0 could not end in a failing run.
#
# This arm drives the PRODUCTION functions, lifted verbatim out of
# run_matrix.sh, rather than a model of them — the ordering being pinned is
# run_matrix.sh's, and a hand-written model would go on passing after a
# production reorder.
cat > "$T/trap27e.sh" <<'TRAP27E'
#!/usr/bin/env bash
set -euo pipefail
# shellcheck source=/dev/null
. "$1"
OUT=$2
SRC=$3
# the real cleanup chain, verbatim from run_matrix.sh
eval "$(sed -n '/^cleanup_snapshot() {/,/^}/p
/^cleanup_results() {/,/^}/p
/^cleanup_ws() {/,/^}/p
/^cleanup_ws_workspace() {/,/^}/p' "$SRC")"
SNAP_ROOT=""
RESULTS_ROOT=""
WS_PARENT_EXPLICIT=""
WS="$OUT/ws"
mkdir -p "$WS"
printf 'what the matrix left behind\n' > "$WS/diagnostic"
WS_ID=$(ws_identity "$WS")
trap cleanup_ws EXIT
ws_mutants_bind "$OUT/tools" "$OUT/tools/cerulion-ros2-migrate-clang" || exit 9
printf 'CORRUPTED\n' > "$OUT/tools/cerulion-ros2-migrate-clang"
rm -f "$WS_MUTANT_PRISTINE"          # the copy-back can no longer succeed
exit 0                               # the BODY succeeds
TRAP27E
chmod 755 "$T/trap27e.sh"
W27H13="$T/w27h13"
mkdir -p "$W27H13/tools"
printf 'PROD\n' > "$W27H13/tools/cerulion-ros2-migrate-clang"
chmod 755 "$W27H13/tools/cerulion-ros2-migrate-clang"
RC27H13=0
timeout 60 bash "$T/trap27e.sh" "$PWD/matrix_lib.sh" "$W27H13" "$PWD/run_matrix.sh" \
    > "$W27H13/out" 2> "$W27H13/err" || RC27H13=$?
# EXACTLY 2 — the promotion's own code — not merely non-zero. MEASURED, and
# it matters: when `set -e` aborts an EXIT trap, bash exits with the FAILING
# COMMAND's status, which is also non-zero. A `-ne 0` oracle therefore passes
# whether the promotion ran or the trap died on the way to it, which is the
# numeric-false-pass class: a bare nonzero check would not catch this.
[ "$RC27H13" -eq 2 ] \
    || fail "the restore-failure promotion did not run (exit $RC27H13, wanted 2): \
$(cat "$W27H13/err")"
[ -d "$W27H13/ws" ] && [ -f "$W27H13/ws/diagnostic" ] \
    || fail "the scratch workspace was DELETED on a run that failed because its restore \
failed — the diagnostics an operator needs are gone before the exit status says to look \
for them"
grep -q "retained because the tool directory could" "$W27H13/out" \
    || fail "the workspace was retained without saying WHY, so an operator reading the run \
cannot tell a restore failure from an ordinary one: $(cat "$W27H13/out")"

# ...and the SAME run with its output streams CLOSED must still fail, and
# still retain. This is the arm for suspending `set -e` across the whole EXIT
# trap. Under `set -euo pipefail` any failing command in a trap aborts the
# rest of it, and reporting IS a failing command when stdout or stderr is
# unwritable (`./run_matrix.sh 2>&1 | head`, a CI `| tee` whose reader
# exited). That turned "exit 2, workspace retained" into "exit 0, workspace
# deleted, corrupted production prover" — silently, and only under a pipe.
# The promotion is the worst of the four sites, because its own `echo` sits
# BEFORE its `exit 2`.
W27H13B="$T/w27h13b"
mkdir -p "$W27H13B/tools"
printf 'PROD\n' > "$W27H13B/tools/cerulion-ros2-migrate-clang"
chmod 755 "$W27H13B/tools/cerulion-ros2-migrate-clang"
RC27H13B=0
timeout 60 bash "$T/trap27e.sh" "$PWD/matrix_lib.sh" "$W27H13B" "$PWD/run_matrix.sh" \
    >&- 2>&- || RC27H13B=$?
# Again EXACTLY 2, and here the distinction IS the whole arm. With the
# suspension removed the retention `echo` fails, `set -e` aborts the trap
# inside cleanup_ws_workspace, and the shell exits 1 — non-zero, and with the
# workspace still on disk because the delete was never reached either. So
# BOTH of the obvious oracles ("it failed", "the workspace survived") are
# satisfied by the broken build. Only the promotion's own code separates
# "the run was failed deliberately" from "the trap died before it could be".
[ "$RC27H13B" -eq 2 ] \
    || fail "with its output streams CLOSED the restore-failure promotion did not run (exit \
$RC27H13B, wanted 2) — a failing report aborted the EXIT trap on its way there, which on a real \
run means exiting without failing it, leaving a corrupted production prover"
[ -d "$W27H13B/ws" ] && [ -f "$W27H13B/ws/diagnostic" ] \
    || fail "with its output streams CLOSED the scratch workspace was DELETED on a run that \
failed because its restore failed"

# (h14) a registered mutant whose REMOVAL FAILS stays REGISTERED, so the
# end-of-run sweep cannot re-report it as a binary "not registered by this
# run". Clearing the registry unconditionally put two contradictory labels on
# one pathname two messages apart, and the second sends an operator hunting a
# build site that skipped its registration when no such site exists.
# Two refusal classes, both UID-INDEPENDENT (a chmod-based injection is
# inert as root and the container runs as root — the ORACLES-vs-ROOT rule):
# a non-empty DIRECTORY at a registered name — never bound, so
# it is refused BEFORE any rm, as never bound — and a BOUND file whose `rm`
# fails, driven with a stub `rm` on PATH that refuses every mutant path (and
# execs the real rm for anything else). Each is planted INSIDE the child,
# AFTER the bind: planting it first trips the STARTUP refusal instead, and
# the arm would then be measuring that rather than the removal path.
# (Sub-arm (h6) learned the same lesson.)
W27H14="$T/w27h14"
mkdir -p "$W27H14/tools" "$W27H14/stub-rm"
printf 'PROD\n' > "$W27H14/tools/cerulion-ros2-migrate-clang"
chmod 755 "$W27H14/tools/cerulion-ros2-migrate-clang"
printf '#!/bin/sh\ncase "$*" in *-mutant-*) echo "rm: cannot remove: Operation not permitted" >&2; exit 1 ;; esac\nexec /bin/rm "$@"\n' \
    > "$W27H14/stub-rm/rm"
chmod 755 "$W27H14/stub-rm/rm"
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
timeout 60 bash -c '. "$1" || exit 9
ws_mutants_bind "$2/tools" "$2/tools/cerulion-ros2-migrate-clang" || exit 9
stuck="$2/tools/cerulion-ros2-migrate-clang-mutant-stuck"
ws_mutant_register "$stuck" || exit 9
mkdir -p "$stuck" || exit 9
: > "$stuck/occupant" || exit 9
ws_mutants_restore' _ "$PWD/matrix_lib.sh" "$W27H14" 2> "$W27H14/err" \
    || fail "the failing-removal child exited non-zero: $(cat "$W27H14/err")"
grep -q "never bound to an identity" "$W27H14/err" \
    || fail "a registered directory (never bound) that the sweep left in place was not reported \
at all: $(cat "$W27H14/err")"
if grep -q "not registered by it" "$W27H14/err"; then
    fail "the sweep re-reported a REGISTERED entry it refused as one \"not registered by it\" \
— the same pathname carries two contradictory labels in one run: $(cat "$W27H14/err")"
fi
rm -rf "$W27H14/tools/cerulion-ros2-migrate-clang-mutant-stuck"
# shellcheck disable=SC2016  # $1/$2/$3 expand in the child shell
timeout 60 bash -c '. "$1" || exit 9
ws_mutants_bind "$2/tools" "$2/tools/cerulion-ros2-migrate-clang" || exit 9
stuck="$2/tools/cerulion-ros2-migrate-clang-mutant-stuck"
ws_mutant_register "$stuck" || exit 9
printf X > "$stuck" || exit 9
ws_mutant_bind_identity "$stuck" || exit 9
PATH="$3:$PATH"
[ "$(command -v rm)" = "$3/rm" ] || exit 9
ws_mutants_restore
[ -f "$stuck" ] || exit 9' _ "$PWD/matrix_lib.sh" "$W27H14" "$W27H14/stub-rm" \
    2> "$W27H14/err2" \
    || fail "the failing-rm child exited non-zero (or the stub rm removed the file): \
$(cat "$W27H14/err2")"
grep -q "could NOT be removed" "$W27H14/err2" \
    || fail "a registered, bound mutant whose rm failed was not reported at all: \
$(cat "$W27H14/err2")"
if grep -q "not registered by it" "$W27H14/err2"; then
    fail "the sweep re-reported a REGISTERED mutant whose removal failed as one \"not registered \
by it\" — the same pathname carries two contradictory labels in one run: $(cat "$W27H14/err2")"
fi
rm -f "$W27H14/tools/cerulion-ros2-migrate-clang-mutant-stuck"

# (f) structural pins — production, not the model above. Counts are data
# here, not verdicts, so `grep -c`'s exit status is deliberately dropped
# (it exits 1 on a zero count, which under `set -e` would kill the selftest
# silently on the very mutation this arm exists to name).
# Deliberately unanchored, so `bash ./build.sh --mutant` is still counted
# (measured). The spelling that
# DOES escape is `"$HERE/build.sh" --mutant`, where the quote breaks the
# substring; that would leave the equality satisfied while a second,
# unregistered build site went uncounted.
# Comment-STRIPPED (the convention for source walks), and matched
# STRUCTURALLY rather than by one spelling. Counting the
# literal `build.sh --mutant ` substring and comparing AGGREGATES
# is blind twice over: `"$HERE/build.sh" --mutant` puts a quote
# between the two tokens so the substring never matches (verified), and an
# aggregate count plus a FIRST-pair ordering check says nothing about a
# second pair. A site invisible to both greps is one the equality happily
# accepts while its mutant binary goes unregistered — exactly the leftover
# this whole arm exists to prevent.
#
# So: every line that invokes build.sh AND carries --mutant, whatever the
# quoting, must be preceded by its own status-checked ws_mutant_register,
# with no other build site in between.
# Shell continuations are joined first.
# Both greps below match within ONE physical line, so an invocation split
# as `./build.sh \` + `--mutant "$name"` carries neither token pair on any
# single line and the site is INVISIBLE: the equality then compares zero
# build sites against zero registrations and passes while an unregistered
# mutant binary is built — the exact leftover this arm exists to prevent.
# MEASURED, and the obvious example is NOT the hole: splitting the
# ONLY build site fails an unjoined walk at its anti-vacuity guard ("found
# no mutant build site in run_matrix.sh"). What an unjoined walk really
# accepts is a SECOND continuation-split unregistered site BESIDE the
# registered one — it counts 1 site against 1 registration and reports
# ALL OK, while the joined walk counts 2 against 1 and fails.
#
# The join is `join_logical_lines`, which
# reproduces bash's splicing rather than approximating it. A join
# on the PRESENCE of a trailing backslash that strips
# comments BEFORE joining looks safe because both divergences
# "push toward MORE joining, never less". They are not: fusing TWO
# BUILD SITES onto one line is also more joining, and `grep -n` then counts
# them ONCE — 2 sites against 1 registration reads as 1-vs-1 and this arm
# passes while a mutant binary goes unregistered. The reasoning now lives on
# the function, where both call sites can see it.
W27_CODE=$(join_logical_lines run_matrix.sh)
W27_BUILD_TEXT=$(printf '%s\n' "$W27_CODE" | grep 'build\.sh' \
    | grep -- '--mutant' || true)
W27_REG_TEXT=$(printf '%s\n' "$W27_CODE" | grep -E 'ws_mutant_register .*\|\|' || true)
W27_BUILD_LINES=$(printf '%s\n' "$W27_CODE" | grep -n 'build\.sh' \
    | grep -- '--mutant' | cut -d: -f1 || true)
W27_REG_LINES=$(printf '%s\n' "$W27_CODE" \
    | grep -nE 'ws_mutant_register .*\|\|' | cut -d: -f1 || true)
# shellcheck disable=SC2206  # deliberate word splitting: these are line numbers
W27_B=($W27_BUILD_LINES)
# shellcheck disable=SC2206
W27_R=($W27_REG_LINES)
[ "${#W27_B[@]}" -ge 1 ] \
    || fail "found no mutant build site in run_matrix.sh — the walk is not reaching the file"
[ "${#W27_B[@]}" -eq "${#W27_R[@]}" ] \
    || fail "${#W27_B[@]} mutant build site(s) against ${#W27_R[@]} status-checked \
registration(s) — an unregistered mutant binary is one the restore cannot remove"
W27_I=0
W27_PREV_BUILD=0
while [ "$W27_I" -lt "${#W27_B[@]}" ]; do
    W27_THIS_B=${W27_B[$W27_I]}
    W27_THIS_R=${W27_R[$W27_I]}
    [ "$W27_THIS_R" -lt "$W27_THIS_B" ] \
        || fail "the mutant build at line $W27_THIS_B is not preceded by its registration \
(line $W27_THIS_R) — an interrupted build leaves a binary nothing recorded"
    [ "$W27_THIS_R" -gt "$W27_PREV_BUILD" ] \
        || fail "the registration at line $W27_THIS_R sits before the PREVIOUS build site \
(line $W27_PREV_BUILD), so one build site is covered twice and another not at all"
    W27_PREV_BUILD=$W27_THIS_B
    W27_I=$((W27_I + 1))
done
# ...and each site must hand the build the SAME `$name` the registration
# chain is derived from, and register the SAME `$mut` that chain produces.
# The chain greps below pin name -> lower -> mut and that A register takes
# "$mut"; not one of them looks at what the BUILD is handed, so changing
# `--mutant "$name"` to any other value leaves the counts, the ordering and
# every grep green while the registered path no longer names the binary
# that gets written — the leftover, reached by renaming rather than by
# skipping the registration. Per SITE, not in aggregate: a second site
# passing something else is the shape an aggregate check cannot see.
# shellcheck disable=SC2016  # the single-quoted patterns are literal source text
while IFS= read -r W27_LINE; do
    [ -n "$W27_LINE" ] || continue
    case $W27_LINE in
        *'--mutant "$name"'*) ;;
        *) fail "a mutant build site hands --mutant something other than \"\$name\", so the \
path registered for restoration need not name the binary this builds: $W27_LINE" ;;
    esac
done <<< "$W27_BUILD_TEXT"
# shellcheck disable=SC2016  # the single-quoted patterns are literal source text
while IFS= read -r W27_LINE; do
    [ -n "$W27_LINE" ] || continue
    case $W27_LINE in
        *'ws_mutant_register "$mut"'*) ;;
        *) fail "a registration names something other than \"\$mut\", so the binary the \
build is about to write is not the one recorded for restoration: $W27_LINE" ;;
    esac
done <<< "$W27_REG_TEXT"
# ...and the registration must name the path the build will WRITE, not some
# other mutant's. Both derive from `$name` inside run_mutant, so the chain is
# what is pinned: name -> lower -> mut, register takes "$mut", build takes
# --mutant "$name".
# shellcheck disable=SC2016  # the single-quoted patterns are literal source text
printf '%s\n' "$W27_CODE" | grep -q 'mut="$WS_MUTANT_ROOT/cerulion-ros2-migrate-clang-mutant-$lower"' \
    || fail "run_matrix.sh no longer derives the registered mutant path from \$lower under the \
run-private root — the registration and the build could name different binaries, or a \
binary at the SHARED name (the build-bind gap)"
# shellcheck disable=SC2016
printf '%s\n' "$W27_CODE" | grep -q 'lower=$(printf' \
    || fail "run_matrix.sh no longer derives \$lower from \$name"
# shellcheck disable=SC2016
printf '%s\n' "$W27_CODE" | grep -q 'ws_mutant_register "$mut"' \
    || fail "run_matrix.sh registers something other than the path it is about to build"

# The production prover must be BOUND AFTER it is BUILT. Checking only that
# the bind appears lets it move above `./build.sh`, and then the pristine copy
# is of the PREVIOUS run's binary: an abort would restore a stale prover, or
# the bind would fail before the new tool exists at all.
W27_PRODBUILD_LINE=$(printf '%s\n' "$W27_CODE" | grep -n 'build\.sh' \
    | grep -v -- '--mutant' | head -1 | cut -d: -f1 || true)
W27_BIND_LINE=$(printf '%s\n' "$W27_CODE" | grep -n 'ws_mutants_bind ' \
    | head -1 | cut -d: -f1 || true)
[ -n "$W27_PRODBUILD_LINE" ] && [ -n "$W27_BIND_LINE" ] \
    || fail "could not locate the production build and the bind in run_matrix.sh"
[ "$W27_BIND_LINE" -gt "$W27_PRODBUILD_LINE" ] \
    || fail "run_matrix.sh binds the production prover (line $W27_BIND_LINE) BEFORE building \
it (line $W27_PRODBUILD_LINE) — the pristine copy would be of the previous run's binary"
sed -n '/^cleanup_ws() {/,/^}/p' run_matrix.sh | grep -q 'ws_mutants_restore' \
    || fail "run_matrix.sh's EXIT trap does not restore the mutants"
W27_TRAPS=$(grep -cE '^[[:space:]]*trap ' run_matrix.sh || true)
[ "$W27_TRAPS" -eq 1 ] \
    || fail "run_matrix.sh installs $W27_TRAPS traps — the EXIT trap is the only one this \
arm drives, so a second trap path would be unpinned coverage"
# The count alone does not pin the SPEC: `trap cleanup_ws INT TERM` keeps the
# count at 1 and the body still restores, while sub-arm (c)'s whole point —
# that the EXIT trap covers the signal path — stops being true of production.
grep -q '^trap cleanup_ws EXIT$' run_matrix.sh \
    || fail "run_matrix.sh's single trap is not 'trap cleanup_ws EXIT' — sub-arm (c) measures \
the EXIT trap's signal coverage, which only means something if EXIT is what is installed"
# shellcheck disable=SC2016  # the single-quoted pattern is literal source text
grep -q 'ws_mutant_register "$mut" || return 2' run_matrix.sh \
    || fail "run_matrix.sh drops ws_mutant_register's status — every run_mutant call site is \
\`if ! run_mutant\`, which disables set -e for the whole call, so a bare register prints its \
refusal and then builds the mutant anyway, unregistered"
# shellcheck disable=SC2016  # the single-quoted patterns are literal source text
grep -q '^ws_mutants_require_clean "$HERE"' run_matrix.sh \
    || fail "run_matrix.sh does not refuse a pre-existing mutant binary before it starts"
# shellcheck disable=SC2016  # the single-quoted pattern is literal source text
grep -q '^ws_mutants_bind "$HERE" "$TOOL"' run_matrix.sh \
    || fail "run_matrix.sh does not bind the production prover it built"
echo "mutant restore OK (an abort by failure AND by signal leaves no mutant binary and an \
unchanged production prover; the no-restore control leaves one behind; a replaced prover is \
restored from the pristine copy; a pre-existing mutant is refused not deleted; every build \
site — shell continuations JOINED — is registered with its status CHECKED and is handed the \
same \$name the registered path derives from; the single trap is 'trap cleanup_ws EXIT'; the \
restore never aborts the trap it runs in, reports what it removed, restores an executable \
prover, reports an unregistered leftover instead of passing in silence, keeps a mutant whose REMOVAL \
FAILED registered so the sweep cannot mislabel it, and a run that failed BECAUSE its restore \
failed retains its scratch workspace and still exits 2 with its output streams CLOSED)"

echo "== selftest 28: -Werror=switch outlives llvm-config's own flags =="
# `-Werror=switch` is what makes the ScanResult dispatch's total switch FAIL
# THE BUILD when a verdict is added without a branch, instead of falling
# through to the rewrite path. Among -W flags clang takes the LAST one, so
# the promotion must sit AFTER `$($LLVM_CONFIG --cxxflags)`: before it, a
# `-Wno-switch` (or a `-w`) from that expansion cancels the promotion while
# build.sh still READS as armed — an inert guard that looks armed. Debian's
# llvm-config 18 emits no -W flags, so nothing opens this today; the ORDER
# is what keeps that from being load-bearing.
#
# Driven through the REAL build.sh with a stub toolchain first on PATH: the
# stub llvm-config emits the hostile `-Wno-switch` from --cxxflags, and the
# stub clang++ records its argv instead of compiling (so this arm needs no
# LLVM and writes nothing into the tool directory). The oracle is the ORDER
# of the two flags in the recorded argv — the property that decides which
# one clang honours. Both stub names are provided because build.sh prefers
# llvm-config-18 and would otherwise find a real one.
B28="$T/bin28"
mkdir -p "$B28" "$T/inc28/clang/Tooling" "$T/lib28"
: > "$T/inc28/clang/Tooling/Tooling.h"
: > "$T/lib28/libclang-cpp.so"
# What the stub injects is read from files at CALL time, so one stub serves
# all three sub-arms below.
: > "$T/inject28_cxx"
: > "$T/inject28_ld"
: > "$T/inject28_lib"
: > "$T/inject28_sys"
cat > "$B28/llvm-config" <<EOF
#!/usr/bin/env bash
case "\$1" in
    --includedir)  echo "$T/inc28" ;;
    --libdir)      echo "$T/lib28" ;;
    --cxxflags)    echo "-I$T/inc28 \$(cat "$T/inject28_cxx")" ;;
    --ldflags)     echo "\$(cat "$T/inject28_ld")" ;;
    --libs)        echo "\$(cat "$T/inject28_lib")" ;;
    --system-libs) echo "\$(cat "$T/inject28_sys")" ;;
    *)             echo "" ;;
esac
EOF
cp "$B28/llvm-config" "$B28/llvm-config-18"
cat > "$B28/clang++" <<EOF
#!/usr/bin/env bash
printf '%s\n' "\$@" > "$T/argv28"
EOF
chmod +x "$B28/llvm-config" "$B28/llvm-config-18" "$B28/clang++"

# (a) ORDER: the promotion must appear LATER in argv than everything
# `--cxxflags` emits, so an ordinary `-Wno-switch` from there is overridden.
# The marker is a -W flag the refusal below does NOT list, because the two
# guards are mutually exclusive by construction: a `-Wno-switch` injected
# here would be REFUSED (sub-arm (d)) and never reach argv at all, so the
# ordering could not be observed. Position is the whole oracle.
printf '%s' '-Wno-unused-variable' > "$T/inject28_cxx"
: > "$T/inject28_ld"; : > "$T/inject28_lib"; : > "$T/inject28_sys"
rm -f "$T/argv28"
( PATH="$B28:$PATH" ./build.sh >/dev/null ) \
    || fail "build.sh failed under the stub toolchain (apparatus broken)"
[ -s "$T/argv28" ] \
    || fail "the stub clang++ recorded no argv — build.sh never reached the compile"
grep -qx -- '-Werror=switch' "$T/argv28" \
    || fail "build.sh does not pass -Werror=switch at all — the ScanResult switch is not promoted"
grep -qx -- '-Wno-unused-variable' "$T/argv28" \
    || fail "the stub llvm-config's --cxxflags marker never reached the command line (apparatus \
broken, so the ordering oracle below would be vacuous)"
W28_ERR=$(grep -nx -- '-Werror=switch' "$T/argv28" | tail -1 | cut -d: -f1)
W28_NO=$(grep -nx -- '-Wno-unused-variable' "$T/argv28" | tail -1 | cut -d: -f1)
[ "$W28_ERR" -gt "$W28_NO" ] \
    || fail "-Werror=switch sits at argv position $W28_ERR, BEFORE the --cxxflags expansion which \
ends at $W28_NO: clang takes the last -W flag, so a -Wno-switch out of that expansion would make \
the promotion INERT while build.sh reads as armed"

# (d) ...and the ordinary `-Wno-switch` out of --cxxflags is refused
# outright, so the ordering above is the second line of defence, not the
# only one.
printf '%s' '-Wno-switch' > "$T/inject28_cxx"
: > "$T/inject28_ld"; : > "$T/inject28_lib"; : > "$T/inject28_sys"
rm -f "$T/argv28"
if OUT28=$( PATH="$B28:$PATH" ./build.sh 2>&1 ); then
    fail "build.sh BUILT a prover with -Wno-switch in llvm-config --cxxflags"
fi
case "$OUT28" in
    *"would make -Werror=switch INERT"*) ;;
    *) fail "the -Wno-switch refusal named the wrong reason: $OUT28" ;;
esac
[ ! -s "$T/argv28" ] \
    || fail "build.sh refused -Wno-switch but still invoked the compiler"

# (b) `-w` is NOT an ordinary -W flag: MEASURED on clang 17, it suppresses in
# EITHER position, so no ordering of the promotion can survive it. The build
# must therefore REFUSE rather than emit a command line whose guard is inert.
printf '%s' '-w' > "$T/inject28_cxx"
: > "$T/inject28_ld"; : > "$T/inject28_lib"; : > "$T/inject28_sys"
rm -f "$T/argv28"
if OUT28=$( PATH="$B28:$PATH" ./build.sh 2>&1 ); then
    fail "build.sh BUILT a prover with '-w' in llvm-config --cxxflags — -w suppresses in either \
position, so -Werror=switch is INERT and a missing ScanResult branch would fall through to the \
rewrite path"
fi
case "$OUT28" in
    *"would make -Werror=switch INERT"*) ;;
    *) fail "the -w refusal named the wrong reason: $OUT28" ;;
esac
[ ! -s "$T/argv28" ] \
    || fail "build.sh refused -w but still invoked the compiler"

# (c) the expansions that land AFTER the promotion on the command line cannot
# be protected by ANY ordering of this one token, so they ride the same
# refusal. Driven over ALL THREE: covering only one would leave dropping
# either of the others from the refusal loop uncaught, in the arm
# whose own standard is that each guard is killed by its own fixture.
: > "$T/inject28_cxx"
for W28_LATE in ld lib sys; do
    : > "$T/inject28_ld"; : > "$T/inject28_lib"; : > "$T/inject28_sys"
    printf '%s' '-Wno-switch' > "$T/inject28_$W28_LATE"
    rm -f "$T/argv28"
    if OUT28=$( PATH="$B28:$PATH" ./build.sh 2>&1 ); then
        fail "build.sh BUILT a prover with -Wno-switch in llvm-config's '$W28_LATE' expansion — \
that expansion lands AFTER -Werror=switch, so the promotion is INERT and no reordering of it \
could help"
    fi
    case "$OUT28" in
        *"would make -Werror=switch INERT"*) ;;
        *) fail "the '$W28_LATE' refusal named the wrong reason: $OUT28" ;;
    esac
    [ ! -s "$T/argv28" ] \
        || fail "build.sh refused the '$W28_LATE' expansion but still invoked the compiler"
done
: > "$T/inject28_ld"; : > "$T/inject28_lib"; : > "$T/inject28_sys"
# (e) `--out-dir DIR` lands the binary at DIR/<the same name>, in
# either flag order, and every malformed spelling is refused BEFORE the
# compiler runs — a caller that asked for a private directory and got the
# shared name would have exactly the build-bind gap the flag exists to
# close. The stub toolchain records argv; `-o` and its operand are the
# oracle. The plain build must still land at the shared name.
: > "$T/inject28_cxx"; : > "$T/inject28_ld"; : > "$T/inject28_lib"; : > "$T/inject28_sys"
mkdir -p "$T/out28"
w28_argv_out() {
    # Echoes the operand that follows `-o` in the recorded argv.
    awk 'p { print; exit } $0 == "-o" { p = 1 }' "$T/argv28"
}
rm -f "$T/argv28"
( PATH="$B28:$PATH" ./build.sh --out-dir "$T/out28" --mutant DROP_X >/dev/null ) \
    || fail "28(e): build.sh refused a valid --out-dir with a mutant (apparatus broken)"
[ "$(w28_argv_out)" = "$T/out28/cerulion-ros2-migrate-clang-mutant-drop_x" ] \
    || fail "28(e): with --out-dir the mutant did not land at DIR/<name>: -o '$(w28_argv_out)'"
rm -f "$T/argv28"
( PATH="$B28:$PATH" ./build.sh --mutant DROP_X --out-dir "$T/out28" >/dev/null ) \
    || fail "28(e): build.sh refused --out-dir AFTER --mutant (apparatus broken)"
[ "$(w28_argv_out)" = "$T/out28/cerulion-ros2-migrate-clang-mutant-drop_x" ] \
    || fail "28(e): flag order changed where the mutant lands: -o '$(w28_argv_out)'"
rm -f "$T/argv28"
( PATH="$B28:$PATH" ./build.sh --out-dir "$T/out28" >/dev/null ) \
    || fail "28(e): build.sh refused --out-dir for the production prover (apparatus broken)"
[ "$(w28_argv_out)" = "$T/out28/cerulion-ros2-migrate-clang" ] \
    || fail "28(e): with --out-dir the production prover did not land at DIR/<name>: -o \
'$(w28_argv_out)'"
rm -f "$T/argv28"
( PATH="$B28:$PATH" ./build.sh >/dev/null ) \
    || fail "28(e): the plain build failed under the stub toolchain (apparatus broken)"
[ "$(w28_argv_out)" = "cerulion-ros2-migrate-clang" ] \
    || fail "28(e): the plain build no longer lands at the shared name: -o '$(w28_argv_out)'"
ln -s "$T/out28" "$T/out28-link"
for W28_BAD in "--out-dir $T/out28-missing" "--out-dir $T/out28-link" \
        "--out-dir $T/out28 --out-dir $T/out28" "--out-dir" "--out-dir $T/out28 --mutant" \
        "--mutant DROP_X --mutant DROP_Y" "--out-dir $T/out28 --bogus" "--out-dir -x"; do
    rm -f "$T/argv28"
    # shellcheck disable=SC2086  # the spelling under test is split on purpose
    if OUT28=$( PATH="$B28:$PATH" ./build.sh $W28_BAD 2>&1 ); then
        fail "28(e): build.sh ACCEPTED '$W28_BAD' — a malformed spelling that must be refused"
    fi
    [ ! -e "$T/argv28" ] \
        || fail "28(e): build.sh refused '$W28_BAD' but still invoked the compiler"
    case "$OUT28" in
        *"error: "*) ;;
        *) fail "28(e): the refusal of '$W28_BAD' does not name its cause: $OUT28" ;;
    esac
done
echo "build guard OK (-Werror=switch at argv $W28_ERR sits after the --cxxflags expansion ending \
at $W28_NO; a -Wno-switch, a -w, and a -Wno-switch out of --system-libs are each REFUSED — -w \
suppresses in either position and the late expansions land after the promotion, so no ordering \
could cover them)"

echo "== selftest 29: the pristine prover is copied THROUGH its descriptor =="
# Gating the pathname's type and then `cp`ing it does not hold the
# gate across the copy. FOUR guards stand here —
# the pathname gate in ws_mutants_bind, plus three in the helper — and each
# is pinned here on its own fixture, so none can mask another:
#   (b,c) the caller's pathname TYPE gate: a FIFO or a symlink sitting at the
#         prover path is refused INSTANTLY and by name, never after the bound;
#   (d)   the helper's own SYMLINK gate — the one property a descriptor
#         cannot report, because open(2) follows the link;
#   (e)   the DESCRIPTOR test, which sees what was actually opened;
#   (f)   the watchdog, which must REFUSE (a status and a diagnostic), not
#         kill its caller;
#   (f2)  a destination that BLOCKS the write, which the bound must still
#         reach — it cannot if the reader is a blocking FOREGROUND child;
#   (g)   the copy-failure branch, which turns a partial/errored write into a
#         refusal instead of a silently short "pristine" copy;
#   (g2)  the same-object refusal: `> "$2"` would truncate the very inode the
#         descriptor is open on and then report success over an empty copy,
#         where `cp a a` refuses outright;
#   (h)   the bytes come from the DESCRIPTOR, not from a second resolution of
#         the pathname — this arm's headline, and the one property the first
#         draft asserted only in prose.
# (a) is the anti-tautology half: without it an implementation that refused
# everything would satisfy every arm below.
#
# One guard here is deliberately UNPINNED and says so: the `[ -s "$1" ] &&
# [ ! -s "$2" ]` empty-copy backstop. Every shape that would reach it is either
# refused earlier (same-object) or not deterministically constructible (a
# source truncated by another writer mid-read leaves `$1` empty too, so the
# test reads false). It is kept as a cheap backstop on this module's worst
# outcome - an empty "pristine" prover the restore would install executable -
# and it masks no other guard's gap.
#
# (d)-(h) call ws_copy_through_descriptor DIRECTLY. That is deliberate: the
# caller's type gate would otherwise refuse the FIFO fixtures before the
# helper's own guards ever ran, and a guard no fixture can reach is a guard
# nothing can verify.
D29="$T/tool29"
mkdir -p "$D29"
printf 'PROVER-BYTES-29' > "$D29/prover"
chmod 0755 "$D29/prover"

# (a) a real prover binds, and the pristine copy is byte-identical.
# shellcheck disable=SC2016  # the child must expand its OWN positional args
P29=$(timeout 30 bash -c '. ./matrix_lib.sh; ws_mutants_bind "$1" "$2" >&2 \
    && printf "%s\n" "$WS_MUTANT_PRISTINE"' _ "$D29" "$D29/prover") \
    || fail "ws_mutants_bind REFUSED a plain regular-file prover"
[ -n "$P29" ] || fail "ws_mutants_bind reported no pristine copy"
[ "$(cat "$P29")" = 'PROVER-BYTES-29' ] \
    || fail "the pristine copy does not carry the prover's bytes: [$(cat "$P29")]"
rm -f "$P29"

# (b) a FIFO at the prover path is refused by the CALLER's type gate, at
# once and by name. The wall matters: without that gate the FIFO reaches the
# descriptor open, blocks, and is TERMed by the watchdog — and bash does NOT
# run an EXIT trap for a TERM delivered inside a builtin (MEASURED), so
# run_matrix.sh's whole cleanup would be skipped with nothing printed.
mkfifo "$D29/prover-fifo-bind"
S29=$SECONDS
# shellcheck disable=SC2016  # the child must expand its OWN positional args
if OUT29=$(timeout 30 bash -c 'WS_GIT_OPEN_TIMEOUT=20; . ./matrix_lib.sh; \
        ws_mutants_bind "$1" "$2"' _ "$D29" "$D29/prover-fifo-bind" 2>&1); then
    fail "ws_mutants_bind ACCEPTED a FIFO as the production prover"
fi
E29=$((SECONDS - S29))
case "$OUT29" in
    *"is not a regular file"*FIFO*) ;;
    *) fail "the FIFO bind refusal named the wrong reason: $OUT29" ;;
esac
[ "$E29" -lt 10 ] \
    || fail "the FIFO bind refusal took ${E29}s against a 20s bound — it came from the WATCHDOG, \
not from the caller's pathname type gate, so a FIFO at the prover path kills the harness (no EXIT \
trap runs for a TERM inside a builtin) instead of being refused"

# (c) ...and a symlink likewise, by the same caller gate.
ln -s prover "$D29/prover-link"
# shellcheck disable=SC2016  # the child must expand its OWN positional args
if OUT29=$(timeout 30 bash -c '. ./matrix_lib.sh; ws_mutants_bind "$1" "$2"' \
        _ "$D29" "$D29/prover-link" 2>&1); then
    fail "ws_mutants_bind ACCEPTED a SYMLINK as the production prover — the pristine copy would \
be an arbitrary target, and the restore would later write it back as 'the prover'"
fi
case "$OUT29" in
    *"is not a regular file"*symlink*) ;;
    *) fail "the symlink bind refusal named the wrong reason: $OUT29" ;;
esac

# (d) the helper's OWN symlink gate, reached directly. The discriminator is
# the gate's own wording: `ws_object_kind` also echoes "symlink", so a looser
# pattern would be satisfied by the DESCRIPTOR branch and a variant that moved
# this gate to AFTER the open — following the link first, which is exactly
# what it exists to prevent — would still pass.
# shellcheck disable=SC2016  # the child must expand its OWN positional args
if OUT29=$(timeout 30 bash -c '. ./matrix_lib.sh; ws_copy_through_descriptor "$1" "$2" "the probe"' \
        _ "$D29/prover-link" "$T/out29d" 2>&1); then
    fail "ws_copy_through_descriptor READ THROUGH a symlink"
fi
case "$OUT29" in
    *"is a SYMLINK before"*) ;;
    *) fail "the direct symlink refusal named the wrong reason: $OUT29" ;;
esac
case "$OUT29" in
    *"DESCRIPTOR is not a regular file"*)
        fail "the symlink was refused by the DESCRIPTOR branch, which means the open had already \
followed the link: $OUT29" ;;
esac

# (e) a FIFO WITH A WRITER opens without blocking, so only the DESCRIPTOR
# test can refuse it.
mkfifo "$D29/prover-fifo"
( sleep 10 > "$D29/prover-fifo" ) &
W29=$!
# shellcheck disable=SC2016  # the child must expand its OWN positional args
if OUT29=$(timeout 30 bash -c '. ./matrix_lib.sh; ws_copy_through_descriptor "$1" "$2" "the probe"' \
        _ "$D29/prover-fifo" "$T/out29e" 2>&1); then
    kill "$W29" 2>/dev/null || true
    fail "ws_copy_through_descriptor ACCEPTED a FIFO with a live writer — the 'pristine copy' is \
then whatever the writer sent"
fi
kill "$W29" 2>/dev/null || true
wait "$W29" 2>/dev/null || true
case "$OUT29" in
    *"DESCRIPTOR is not a regular file"*) ;;
    *) fail "the FIFO descriptor refusal named the wrong reason: $OUT29" ;;
esac
case "$OUT29" in
    *"is a SYMLINK"*) fail "the FIFO was refused by the symlink gate: $OUT29" ;;
esac

# (f) a WRITERLESS FIFO blocks in open(2). The helper must REFUSE within the
# bound — a status AND a diagnostic — rather than kill its caller: the
# watchdog targets the SUBSHELL's pid, not the caller's. `rc == 2` (not 143)
# is what separates the two, and the wall is asserted on BOTH sides of the
# bound: below it the bound is not being honoured, far above it the
# WS_GIT_OPEN_TIMEOUT knob has gone inert.
mkfifo "$D29/prover-fifo-nw"
S29=$SECONDS
# shellcheck disable=SC2016  # the child must expand its OWN positional args
# `set -euo pipefail` in the child is load-bearing, not decoration: these
# direct drives run in `bash -c`, which does NOT inherit the harness's options,
# so without it the refusal below is proven only under a shell mode production
# never uses — and a bare `( … )` in the helper is an errexit TRIGGER that
# would kill the caller before its `case` could refuse.
if OUT29=$(timeout 40 bash -c 'set -euo pipefail; WS_GIT_OPEN_TIMEOUT=3; \
        . ./matrix_lib.sh; ws_copy_through_descriptor "$1" "$2" "the probe"' \
        _ "$D29/prover-fifo-nw" "$T/out29f" 2>&1); then
    fail "ws_copy_through_descriptor ACCEPTED a writerless FIFO"
else
    RC29=$?
fi
E29=$((SECONDS - S29))
[ "$RC29" -ne 124 ] \
    || fail "ws_copy_through_descriptor HUNG on a writerless FIFO (killed by timeout) — the bind \
runs before the matrix, so an unbounded open wedges the whole gate"
[ "$RC29" -eq 2 ] \
    || fail "the writerless FIFO gave status $RC29, not a refusal (2) — the watchdog killed the \
CALLER instead of the subshell it guards, and a TERM delivered inside a builtin runs no EXIT trap"
case "$OUT29" in
    *"within the bound"*) ;;
    *) fail "the writerless FIFO refusal produced no bound diagnostic: [$OUT29]" ;;
esac
[ "$E29" -ge 3 ] \
    || fail "the writerless-FIFO refusal returned in ${E29}s against a 3s bound — it did not come \
from the watchdog"
[ "$E29" -lt 15 ] \
    || fail "the writerless-FIFO refusal took ${E29}s against a 3s bound — WS_GIT_OPEN_TIMEOUT is \
not the bound actually being honoured"

# (f2) a destination that BLOCKS the write (a writerless FIFO) must also
# refuse within the bound. A trap cannot interrupt a child blocked in its OWN
# redirection, so `cat > "$2"` made the subshell wait forever and the bound
# never fired at all (measured). Both ends are therefore opened
# by the subshell the watchdog guards — an open is a builtin there and a TERM
# interrupts it. Returning AT ALL is the observable the `cat > "$2"` shape
# fails.
mkfifo "$D29/dest-fifo-nw"
S29=$SECONDS
# shellcheck disable=SC2016  # the child must expand its OWN positional args
if OUT29=$(timeout 40 bash -c 'set -uo pipefail; WS_GIT_OPEN_TIMEOUT=3; \
        . ./matrix_lib.sh; ws_copy_through_descriptor "$1" "$2" "the probe"' \
        _ "$D29/prover" "$D29/dest-fifo-nw" 2>&1); then
    fail "ws_copy_through_descriptor reported SUCCESS writing into a destination that blocks"
else
    RC29=$?
fi
E29=$((SECONDS - S29))
[ "$RC29" -ne 124 ] \
    || fail "ws_copy_through_descriptor HUNG on a blocking DESTINATION (killed by timeout) — a \
trap cannot interrupt a child blocked in its OWN redirection, so BOTH ends must be opened by the \
subshell the watchdog guards, never by \`cat\`"
[ "$RC29" -eq 2 ] \
    || fail "the blocking destination gave status $RC29, not a refusal (2)"
[ "$E29" -lt 15 ] \
    || fail "the blocking-destination refusal took ${E29}s against a 3s bound"

# (g) a copy that cannot be WRITTEN must refuse, not return a short
# "pristine" copy the restore would later install over the real prover.
# shellcheck disable=SC2016  # the child must expand its OWN positional args
if OUT29=$(timeout 30 bash -c '. ./matrix_lib.sh; ws_copy_through_descriptor "$1" "$2" "the probe"' \
        _ "$D29/prover" "$T/nonexistent-dir-29/dst" 2>&1); then
    fail "ws_copy_through_descriptor reported SUCCESS for a copy it could not write"
fi
case "$OUT29" in
    *"failed before the probe"*) ;;
    *) fail "the copy-failure refusal named the wrong reason: $OUT29" ;;
esac

# (g2) destination == source must REFUSE, not truncate the source through the
# descriptor already open on it and then report success over an empty copy.
# `cp a a` refuses ("are identical"); `> "$2"` does not, so the equivalence the
# docstring draws with `cp` has to be guarded rather than asserted.
printf 'SELF-BYTES-29' > "$D29/self-src"
# shellcheck disable=SC2016  # the child must expand its OWN positional args
if OUT29=$(timeout 30 bash -c 'set -euo pipefail; . ./matrix_lib.sh; \
        ws_copy_through_descriptor "$1" "$1" "the probe"' \
        _ "$D29/self-src" 2>&1); then
    fail "ws_copy_through_descriptor reported SUCCESS copying a file onto ITSELF"
fi
case "$OUT29" in
    *"SAME OBJECT"*) ;;
    *) fail "the same-object refusal named the wrong reason: $OUT29" ;;
esac
[ "$(cat "$D29/self-src")" = 'SELF-BYTES-29' ] \
    || fail "the same-object refusal still TRUNCATED the source: [$(cat "$D29/self-src")]"

# (h) THE HEADLINE: the bytes come from the DESCRIPTOR. The pathname is
# swapped for a different regular file while the helper is parked between
# its open and its read, so a second resolution of the name would copy the
# HOSTILE bytes. Deterministic — two FIFOs, no timing.
P29D=$(pause_dir_new 29)
printf 'ORIGINAL-BYTES-29' > "$D29/swap-src"
printf 'HOSTILE-BYTES-29!' > "$T/hostile29"
(
    # shellcheck disable=SC2016  # the child must expand its OWN positional args
    CERULION_WS_COPY_PAUSE_DIR="$P29D" WS_GIT_OPEN_TIMEOUT=60 \
        timeout 60 bash -c '. ./matrix_lib.sh; \
            ws_copy_through_descriptor "$1" "$2" "the swap probe"' \
        _ "$D29/swap-src" "$T/swap-dst29" > "$T/swap-out29" 2>&1
    echo "$?" > "$T/swap-rc29"
) &
SWAP29=$!
# The BOUNDED handshake (pause_dir_new/wait_paused/release_paused), not a raw
# `read < fifo`: opening a writerless FIFO for reading BLOCKS rather than
# failing, so a raw open's `|| fail` is unreachable and any regression that
# refuses BEFORE the seam wedges the whole selftest with no diagnostic instead
# of failing this arm. MEASURED with the seam made inert:
# rc 124 under an external timeout, i.e. forever without one. Every other
# pause-driving arm in this file already uses these helpers for exactly the
# reason their own comment states.
wait_paused "$P29D" "the copy pause seam"
mv "$T/hostile29" "$D29/swap-src"
release_paused "$P29D"
wait "$SWAP29" 2>/dev/null || true
[ "$(cat "$T/swap-rc29")" = 0 ] \
    || fail "the swap probe did not complete: rc=$(cat "$T/swap-rc29") [$(cat "$T/swap-out29")]"
[ "$(cat "$T/swap-dst29")" = 'ORIGINAL-BYTES-29' ] \
    || fail "the copy followed the PATHNAME, not the descriptor it checked: the destination carries \
[$(cat "$T/swap-dst29")] after the name was swapped mid-copy, so the type gate is not held across \
the read and the original TOCTOU is back"

echo "prover snapshot OK (a real prover binds byte-identically; a FIFO and a symlink at the prover \
path are refused instantly by name; the helper's symlink gate, descriptor test, watchdog, \
copy-failure branch and same-object refusal are each driven by a DIFFERENT fixture; the watchdog \
refuses rather than killing its caller, under the harness's own shell options; and a pathname \
swapped mid-copy does not change the bytes)"

echo "== selftest 30: the continuation join is bash-faithful =="
# join_logical_lines feeds every source walk above. A join that FUSES two
# commands makes `grep -n` count two build sites ONCE, so 2 sites against 1
# registration reads as 1-vs-1 and arm 27 passes while a mutant binary goes
# unregistered — the exact leftover arm 27 exists to prevent. Each fixture
# below is a shape a presence-based join gets wrong, with a hand-written count;
# every expectation was checked against a real bash first.
w30_sites() {
    # $1 fixture file. Echoes the number of joined lines that are a mutant
    # build site — arm 27's own counting, one walk in. The join's status is
    # captured SEPARATELY: piping it straight into grep would let a broken
    # awk read as "0 sites", which the one fixture expecting 0 would accept.
    # NOTE: this runs inside a command substitution at every call site, so a
    # `fail` here would exit only the SUBSTITUTION. Echo a sentinel the integer
    # comparison cannot accept instead, so the caller's own message is what the
    # operator sees.
    local joined
    [ -s "$1" ] || { printf 'MISSING-FIXTURE\n'; return 0; }
    joined=$(join_logical_lines "$1") || { printf 'JOIN-FAILED\n'; return 0; }
    printf '%s\n' "$joined" | grep 'build\.sh' | grep -c -- '--mutant' || true
}
# An EVEN backslash run is an escaped backslash, not a continuation: bash
# runs two commands here, and a presence-based join fuses them into one.
cat > "$T/j30a" <<'J30A'
./build.sh --mutant "$name" \\
./build.sh --mutant "$name"
J30A
[ "$(w30_sites "$T/j30a")" -eq 2 ] \
    || fail "two build sites separated by an ESCAPED backslash were fused into $(w30_sites "$T/j30a") \
site(s) — a second, unregistered build site is invisible to arm 27"
# A comment spliced into a continuation ENDS the command at its `#`; the
# line after it is a new command. Stripping comments before joining fused
# the two commands on either side.
cat > "$T/j30b" <<'J30B'
./build.sh --mutant "$name" \
# a comment between the two sites
./build.sh --mutant "$name"
J30B
[ "$(w30_sites "$T/j30b")" -eq 2 ] \
    || fail "two build sites separated by a comment line were fused into $(w30_sites "$T/j30b") site(s)"
# A full-line comment ending in a backslash does NOT continue (measured):
# the command below it must survive. A join that stripped comments only
# AFTER joining would swallow it into the comment and count zero.
cat > "$T/j30c" <<'J30C'
# a comment ending in a backslash \
./build.sh --mutant "$name"
J30C
[ "$(w30_sites "$T/j30c")" -eq 1 ] \
    || fail "a build site below a comment line ending in a backslash was swallowed \
($(w30_sites "$T/j30c") site(s) found) — the walk would miss it entirely"
# ...and the property the join exists for must still hold: a
# GENUINE continuation is joined, so the split site is seen at all. Without
# this a join that never joined would satisfy every fixture above.
cat > "$T/j30d" <<'J30D'
./build.sh \
    --mutant "$name"
J30D
[ "$(w30_sites "$T/j30d")" -eq 1 ] \
    || fail "a genuinely continued build site was not joined ($(w30_sites "$T/j30d") site(s) found) \
— the continuation-join hole is reopened: neither token pair lands on one physical line"
# A comment spliced into a continuation ends the command at its `#`, so a
# build site MENTIONED inside that comment is not a site. Appending the
# comment text instead can only ever ADD matches, which is the safe
# direction — but it is still a count this walk would have to explain.
cat > "$T/j30e" <<'J30E'
echo hello \
# ./build.sh --mutant "$name" used to live here
./build.sh --mutant "$name"
J30E
[ "$(w30_sites "$T/j30e")" -eq 1 ] \
    || fail "a build site MENTIONED in a comment spliced into a continuation was counted as a \
real site ($(w30_sites "$T/j30e") found, 1 real)"
# A TRAILING comment ends the command too, so a backslash INSIDE it is not a
# continuation — bash runs two commands here (measured). A join that tests
# the raw line for a trailing backslash splices through the comment and
# counts the two sites ONCE, which is the same 2-vs-1-reads-as-1-vs-1 hole
# as fixture (a), reached from a different comment position.
cat > "$T/j30g" <<'J30G'
./build.sh --mutant "$name" --extra # a note ending in a backslash \
./build.sh --mutant "$name"
J30G
[ "$(w30_sites "$T/j30g")" -eq 2 ] \
    || fail "two build sites separated by a TRAILING comment ending in a backslash were fused into \
$(w30_sites "$T/j30g") site(s) — the continuation test is reading the comment, not the code"
# An ESCAPED SPACE before a `#` is not a comment separator, so the `#` is not
# a comment and the line does not end in a continuation — bash runs TWO
# commands (measured). A trailing-regex strip removes the "comment", leaves a
# lone backslash, and the join FUSES them: an undercount, the unsafe
# direction, and exactly what this walk exists to prevent.
cat > "$T/j30h" <<'J30H'
./build.sh --mutant "$name" \ # a note after an escaped space
./build.sh --mutant "$name"
J30H
[ "$(w30_sites "$T/j30h")" -eq 2 ] \
    || fail "two build sites separated by an ESCAPED SPACE before a '#' were fused into \
$(w30_sites "$T/j30h") site(s) — the comment strip treated an escaped space as a separator and \
left a trailing backslash behind"
# ...and a `#` INSIDE QUOTES is not a comment at all, so nothing after it on
# the line may be discarded.
# The build site must be on the SAME line as the quoted `#`, or the count is
# identical either way and a quote-blind strip survives the fixture.
cat > "$T/j30i" <<'J30I'
echo "a # b" ; ./build.sh --mutant "$name"
J30I
[ "$(w30_sites "$T/j30i")" -eq 1 ] \
    || fail "a quoted '#' swallowed the build site after it ($(w30_sites "$T/j30i") found, 1 \
real) — the comment strip is treating quoted text as a comment"
# ...and the same quoted `#` on a line that IS a continuation. This is the
# shape a quote-blind strip hides most completely: it truncates the line at
# the quoted `#`, the trailing backslash goes with it, the join never
# happens, and NEITHER emitted line carries both tokens — so the site
# disappears from the walk entirely rather than merely being miscounted.
# Distinct from the fixture above, where the site sits on the same line.
cat > "$T/j30j" <<'J30J'
./build.sh --tag "sha # $x" \
    --mutant "$name"
J30J
[ "$(w30_sites "$T/j30j")" -eq 1 ] \
    || fail "a quoted '#' on a CONTINUED line hid the build site entirely ($(w30_sites "$T/j30j") \
found, 1 real) — the strip truncated the line at the quoted '#', taking the continuation backslash \
with it, so neither half carries both tokens"
# ...and a plain full-line comment is still dropped: this is the whole of
# the convention the walk inherited when the strip moved inside the
# join, and without it every commented-out build site counts.
cat > "$T/j30f" <<'J30F'
# ./build.sh --mutant "$name" is only mentioned here
echo hello
J30F
[ "$(w30_sites "$T/j30f")" -eq 0 ] \
    || fail "a build site mentioned in a plain comment was counted as a real site \
($(w30_sites "$T/j30f") found, 0 real) — the join no longer strips comments at all"
echo "join fidelity OK (escaped backslash ends the line, a spliced comment ends the command, a \
comment never swallows the line below it, a real continuation is still joined, a trailing comment is not code, an escaped space \
and a quoted '#' are not comment starts, and a site mentioned in a comment is not a site)"

echo "== selftest 31: the mutant registry binds the ARTIFACT, not the pathname =="
# Removing a registered mutant by NAME is unsafe: the per-mutant cleanup in
# run_mutant and the EXIT-trap sweep would delete whatever the name denotes at
# that instant, so with two runs over one tool directory run 1's cleanup
# deletes run 2's live mutant and corrupts its verdict. The registry
# binds dev:inode:size:mtime(ns) at build time and every removal refuses a
# name that no longer carries it. Each guard is driven DIRECTLY through
# matrix_lib.sh in a child bash — bind the prover, register the mutant,
# bind its identity, TAMPER, then remove or restore — one fixture per
# guard, and every tamper asserts its own PRECONDITION (the identity really
# changed) so an arm cannot pass on a filesystem that could not see it.
w31_tool() {
    # $1 tag. A fresh tool dir holding a PROD prover; echoes the dir. The
    # mutant itself is written by the CHILD, after the bind — the bind's
    # startup gate refuses a mutant that already exists, exactly as it
    # refuses a leftover, so the fixture "builds" it when production does.
    local d="$T/w31-$1"
    mkdir -p "$d/tools"
    printf 'PROD\n' > "$d/tools/cerulion-ros2-migrate-clang"
    chmod 755 "$d/tools/cerulion-ros2-migrate-clang"
    printf '%s\n' "$d/tools"
}
# The child prelude every sub-arm shares, in production's order: source the
# lib, bind the prover, register the mutant, "build" it, bind its identity.
# `$M` is the mutant's path. `w31_raw` is the arm's OWN reading of
# dev:inode:size:mtime(ns) — an independent copy, deliberately not
# `ws_file_identity`, so a tamper's precondition below cannot be satisfied
# or defeated by the function under test (a variant that weakened the key
# would otherwise fail the precondition instead of reaching the oracle).
# `W31_RAW_PRE` is that reading BEFORE the bind and `W31_RAW0` after it: the
# bind stamps the mtime, so the two differ by exactly the stamp,
# which sub-arm (n) models a coalesced clock against. The prelude also pins
# PARITY: the identity the bind took through its descriptor
# (python fstat) must equal what this host's stat renders for the name —
# the verify and the removals read by name, so a rendering drift between
# the two would refuse every artifact this run built.
# shellcheck disable=SC2016  # $1/$2/$M expand in the child shell
W31_PRELUDE='. "$1" || exit 9
w31_raw() { stat -c "%d:%i:%s:%.9Y" "$1" 2>/dev/null || stat -f "%d:%i:%z:%Fm" "$1"; }
ws_mutants_bind "$2" "$2/cerulion-ros2-migrate-clang" || exit 9
M="$2/cerulion-ros2-migrate-clang-mutant-x"
ws_mutant_register "$M" || exit 9
printf "MUTANT-BYTES\n" > "$M" || exit 9
chmod 755 "$M" || exit 9
W31_RAW_PRE=$(w31_raw "$M") || exit 9
ws_mutant_bind_identity "$M" || exit 9
W31_RAW0=$(w31_raw "$M") || exit 9
[ "$(ws_mutant_identity_of "$M")" = "$W31_RAW0" ] || { echo "PRECONDITION: the identity bound through the descriptor ($(ws_mutant_identity_of "$M")) is not what this host renders for the name through stat ($W31_RAW0) — the two readings must agree byte for byte or every verify would refuse"; exit 9; }
'
# A tamper's precondition: the object at the name must now READ differently
# (dev:inode:size:mtime, by the arm's own stat) than it did when bound, or
# the sub-arm proves nothing on this filesystem.
# shellcheck disable=SC2016  # $M/$W31_RAW0 expand in the child shell
W31_CHANGED='[ "$(w31_raw "$M")" != "$W31_RAW0" ] || { echo "PRECONDITION: the tamper changed nothing this filesystem can see"; exit 9; }
'

# (a) the untouched artifact is removed and UNREGISTERED. The unregister is
# observable only through what happens NEXT: a later occupant of the same
# name must be reported by the sweep as "not registered by this run", never
# refused as a swap of an artifact this run no longer holds — and it must
# not latch a restore failure for an object this run never built.
W31A=$(w31_tool a)
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
OUT31A=$(timeout 30 bash -c "$W31_PRELUDE"'
ws_mutant_remove "$M" || { echo "REMOVE-REFUSED"; exit 9; }
[ ! -e "$M" ] || { echo "STILL-THERE"; exit 9; }
printf "A LATER OCCUPANT\n" > "$M" || exit 9
ws_mutants_restore
if ws_mutants_restore_failed; then echo LATCHED; else echo CLEAN; fi' \
    _ "$PWD/matrix_lib.sh" "$W31A" 2>&1) \
    || fail "31(a): the untouched-artifact child failed before its oracle: $OUT31A"
case "$OUT31A" in
    *"NOT the artifact this run built"*) fail "31(a): the removal did not UNREGISTER — the \
sweep still held the removed name and refused a LATER occupant as a swap of its own artifact: \
$OUT31A" ;;
esac
case "$OUT31A" in
    *CLEAN*) ;;
    *) fail "31(a): a name this run removed and then another occupied was latched as THIS \
run's restore failure: $OUT31A" ;;
esac
case "$OUT31A" in
    *"survived this run and were not registered by it"*) ;;
    *) fail "31(a): the later occupant was not reported by the not-registered sweep: $OUT31A" ;;
esac
[ "$(cat "$W31A/cerulion-ros2-migrate-clang-mutant-x")" = "A LATER OCCUPANT" ] \
    || fail "31(a): the sweep DELETED a later occupant of a name this run had already removed"

# (b) THE HEADLINE: the artifact behind the name is replaced (new inode,
# different bytes). The per-mutant removal must refuse, the EXIT-trap sweep
# must refuse AND latch a restore failure, and the object must survive both.
W31B=$(w31_tool b)
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
OUT31B=$(timeout 30 bash -c "$W31_PRELUDE"'
rm -f "$M" || exit 9
printf "[HOSTILE-31B]\n" > "$M" || exit 9
'"$W31_CHANGED"'
if ws_mutant_remove "$M"; then echo "REMOVE-ACCEPTED"; fi
if [ -e "$M" ]; then echo "SURVIVED-REMOVE"; fi
ws_mutants_restore
if [ -e "$M" ]; then echo "SURVIVED-RESTORE"; fi
if ws_mutants_restore_failed; then echo LATCHED; else echo CLEAN; fi' \
    _ "$PWD/matrix_lib.sh" "$W31B" 2>&1) \
    || fail "31(b): the swapped-artifact child failed before its oracle: $OUT31B"
case "$OUT31B" in
    *REMOVE-ACCEPTED*) fail "31(b): ws_mutant_remove ACCEPTED a name whose artifact was \
replaced (new inode, different bytes) — the registry still keys on the pathname: $OUT31B" ;;
esac
case "$OUT31B" in
    *SURVIVED-REMOVE*) ;;
    *) fail "31(b): the swapped artifact was DELETED by ws_mutant_remove: $OUT31B" ;;
esac
case "$OUT31B" in
    *SURVIVED-RESTORE*) ;;
    *) fail "31(b): the swapped artifact was DELETED by the EXIT-trap sweep: $OUT31B" ;;
esac
case "$OUT31B" in
    *LATCHED*) ;;
    *) fail "31(b): a registered name carrying a foreign artifact did not FAIL the run \
(the restore failure was not latched): $OUT31B" ;;
esac
case "$OUT31B" in
    *"NOT the artifact this run built"*) ;;
    *) fail "31(b): the refusal does not name its cause: $OUT31B" ;;
esac
[ "$(cat "$W31B/cerulion-ros2-migrate-clang-mutant-x")" = "[HOSTILE-31B]" ] \
    || fail "31(b): the object at the name afterwards is not the swapped-in one"

# (c) a BYTE-IDENTICAL copy renamed over the name (new inode, same bytes)
# is refused too: the key is the artifact's identity, not its content. A
# content digest would accept this — and a peer run's rebuild of the same
# mutant is exactly this shape.
W31C=$(w31_tool c)
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
OUT31C=$(timeout 30 bash -c "$W31_PRELUDE"'
cp "$M" "$M.copy" || exit 9
mv -f "$M.copy" "$M" || exit 9
[ "$(cat "$M")" = "MUTANT-BYTES" ] || { echo "PRECONDITION: bytes changed"; exit 9; }
'"$W31_CHANGED"'
if ws_mutant_remove "$M"; then echo "REMOVE-ACCEPTED"; fi
if [ -e "$M" ]; then echo SURVIVED-REMOVE; fi
ws_mutants_restore
if [ -e "$M" ]; then echo SURVIVED-SWEEP; fi
if ws_mutants_restore_failed; then echo LATCHED; else echo CLEAN; fi' \
    _ "$PWD/matrix_lib.sh" "$W31C" 2>&1) \
    || fail "31(c): the same-bytes-swap child failed before its oracle: $OUT31C"
case "$OUT31C" in
    *REMOVE-ACCEPTED*) fail "31(c): a byte-identical file under a NEW INODE was accepted as \
this run's artifact — the registry keys on content, not identity: $OUT31C" ;;
esac
case "$OUT31C" in
    *SURVIVED-REMOVE*) ;;
    *) fail "31(c): the same-bytes replacement was DELETED by the removal: $OUT31C" ;;
esac
case "$OUT31C" in
    *SURVIVED-SWEEP*) ;;
    *) fail "31(c): the same-bytes replacement was DELETED by the sweep: $OUT31C" ;;
esac
case "$OUT31C" in
    *LATCHED*) ;;
    *) fail "31(c): a same-bytes swap did not FAIL the run: $OUT31C" ;;
esac

# (d) an IN-PLACE rewrite of the same SIZE (same inode, same length, new
# mtime) is refused: the nanosecond mtime is part of the key. This is the
# shape a linker that truncates its output in place leaves behind.
W31D=$(w31_tool d)
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
OUT31D=$(timeout 30 bash -c "$W31_PRELUDE"'
printf "MUTANT-BYTEZ\n" > "$M" || exit 9
[ "$(cat "$M")" = "MUTANT-BYTEZ" ] || { echo "PRECONDITION: rewrite did not land"; exit 9; }
[ "$(w31_raw "$M" | cut -d: -f1,2)" = "$(printf %s "$W31_RAW0" | cut -d: -f1,2)" ] || { echo "PRECONDITION: the inode changed — not an in-place rewrite"; exit 9; }
[ "$(w31_raw "$M" | cut -d: -f3)" = "$(printf %s "$W31_RAW0" | cut -d: -f3)" ] || { echo "PRECONDITION: the size changed"; exit 9; }
'"$W31_CHANGED"'
if ws_mutant_remove "$M"; then echo "REMOVE-ACCEPTED"; fi
if [ -e "$M" ]; then echo SURVIVED-REMOVE; fi
ws_mutants_restore
if [ -e "$M" ]; then echo SURVIVED-SWEEP; fi
if ws_mutants_restore_failed; then echo LATCHED; else echo CLEAN; fi' \
    _ "$PWD/matrix_lib.sh" "$W31D" 2>&1) \
    || fail "31(d): the in-place-rewrite child failed before its oracle: $OUT31D"
case "$OUT31D" in
    *REMOVE-ACCEPTED*) fail "31(d): a same-inode, same-size in-place rewrite was accepted as \
this run's artifact — the mtime is not part of the key: $OUT31D" ;;
esac
case "$OUT31D" in
    *SURVIVED-REMOVE*) ;;
    *) fail "31(d): the rewritten object was DELETED by the removal: $OUT31D" ;;
esac
case "$OUT31D" in
    *SURVIVED-SWEEP*) ;;
    *) fail "31(d): the rewritten object was DELETED by the sweep: $OUT31D" ;;
esac
case "$OUT31D" in
    *LATCHED*) ;;
    *) fail "31(d): a same-size in-place rewrite did not FAIL the run: $OUT31D" ;;
esac

# (e) a mutant-named file this run never REGISTERED is refused by name, and
# survives — the never-remove-what-we-did-not-create rule, at the seam.
W31E=$(w31_tool e)
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
OUT31E=$(timeout 30 bash -c "$W31_PRELUDE"'
printf "SOMEONE ELSES\n" > "$2/cerulion-ros2-migrate-clang-mutant-other" || exit 9
if ws_mutant_remove "$2/cerulion-ros2-migrate-clang-mutant-other"; then echo "REMOVE-ACCEPTED"; fi' \
    _ "$PWD/matrix_lib.sh" "$W31E" 2>&1) \
    || fail "31(e): the unregistered-path child failed before its oracle: $OUT31E"
case "$OUT31E" in
    *REMOVE-ACCEPTED*) fail "31(e): ws_mutant_remove removed a path this run never \
registered: $OUT31E" ;;
esac
case "$OUT31E" in
    *"was not registered by this run"*) ;;
    *) fail "31(e): the unregistered-path refusal does not name its cause: $OUT31E" ;;
esac
[ -e "$W31E/cerulion-ros2-migrate-clang-mutant-other" ] \
    || fail "31(e): a mutant this run never registered was DELETED"

# (f) a registered entry whose build never completed — no identity bound —
# is REFUSED, named and latched by the sweep, and refused (rc 2) by the
# seam: the rule at every removal site is no identity ⇒ no removal.
# Sweeping it BLIND is the
# destructive class the build-bind gap makes concrete: an object this run
# cannot prove is its own. The anti-vacuity half: the same entry with
# NOTHING at the name — a compiler that produced no output — is not a
# failure (nothing to name, nothing latched).
W31F=$(w31_tool f)
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
OUT31F=$(timeout 30 bash -c '. "$1" || exit 9
ws_mutants_bind "$2" "$2/cerulion-ros2-migrate-clang" || exit 9
ws_mutant_register "$2/cerulion-ros2-migrate-clang-mutant-x" || exit 9
printf "HALF-WRITTEN\n" > "$2/cerulion-ros2-migrate-clang-mutant-x" || exit 9
if ws_mutant_remove "$2/cerulion-ros2-migrate-clang-mutant-x"; then echo "SEAM-REMOVED-UNBOUND"; fi
if [ -e "$2/cerulion-ros2-migrate-clang-mutant-x" ]; then echo SURVIVED-SEAM; fi
ws_mutants_restore
if [ -e "$2/cerulion-ros2-migrate-clang-mutant-x" ]; then echo SURVIVED-SWEEP; fi
if ws_mutants_restore_failed; then echo LATCHED; else echo CLEAN; fi' \
    _ "$PWD/matrix_lib.sh" "$W31F" 2>&1) \
    || fail "31(f): the never-bound-entry child failed before its oracle: $OUT31F"
case "$OUT31F" in
    *SEAM-REMOVED-UNBOUND*) fail "31(f): ws_mutant_remove REMOVED a registered entry with NO \
bound identity — no identity ⇒ no removal: $OUT31F" ;;
esac
case "$OUT31F" in
    *SURVIVED-SEAM*) ;;
    *) fail "31(f): the seam DELETED a never-bound entry (the blind arm is back): $OUT31F" ;;
esac
case "$OUT31F" in
    *SURVIVED-SWEEP*) ;;
    *) fail "31(f): the sweep DELETED a never-bound entry (swept blind): $OUT31F" ;;
esac
case "$OUT31F" in
    *LATCHED*) ;;
    *) fail "31(f): a never-bound entry left in place did not FAIL the run: $OUT31F" ;;
esac
W31F_NAMED=$(printf '%s\n' "$OUT31F" | grep -c "never bound to an identity" || true)
[ "$W31F_NAMED" -eq 2 ] \
    || fail "31(f): expected the never-bound refusal from the seam AND from the sweep (2), \
got $W31F_NAMED: $OUT31F"
case "$OUT31F" in
    *"$W31F/cerulion-ros2-migrate-clang-mutant-x"*) ;;
    *) fail "31(f): the refusal does not NAME the path it left in place: $OUT31F" ;;
esac
rm -f "$W31F/cerulion-ros2-migrate-clang-mutant-x"
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
OUT31F2=$(timeout 30 bash -c '. "$1" || exit 9
ws_mutants_bind "$2" "$2/cerulion-ros2-migrate-clang" || exit 9
ws_mutant_register "$2/cerulion-ros2-migrate-clang-mutant-x" || exit 9
ws_mutant_remove "$2/cerulion-ros2-migrate-clang-mutant-x" || { echo "ABSENT-REFUSED"; exit 9; }
ws_mutant_register "$2/cerulion-ros2-migrate-clang-mutant-x" || exit 9
ws_mutants_restore
if ws_mutants_restore_failed; then echo LATCHED; else echo CLEAN; fi' \
    _ "$PWD/matrix_lib.sh" "$W31F" 2>&1) \
    || fail "31(f): the absent-never-bound child failed before its oracle: $OUT31F2"
case "$OUT31F2" in
    *CLEAN*) ;;
    *) fail "31(f): a never-bound entry with NOTHING at its name latched a failure — a build \
that produced no output is not a leftover: $OUT31F2" ;;
esac

# (g) the pre-run verify: passes the bound artifact, refuses a swapped one,
# refuses an entry that was never bound. This is what run_mutant calls
# immediately before EXECUTING the mutant, so a verdict is never taken over
# an object that is not the one this run built.
W31G=$(w31_tool g)
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
OUT31G=$(timeout 30 bash -c "$W31_PRELUDE"'
ws_mutant_verify_identity "$M" "run it (untouched)" || { echo "UNTOUCHED-REFUSED"; exit 9; }
rm -f "$M" && printf "SWAPPED\n" > "$M" || exit 9
'"$W31_CHANGED"'
if ws_mutant_verify_identity "$M" "run it (swapped)"; then echo "SWAPPED-ACCEPTED"; fi
ws_mutant_register "$2/cerulion-ros2-migrate-clang-mutant-unbound" || exit 9
printf "X\n" > "$2/cerulion-ros2-migrate-clang-mutant-unbound" || exit 9
if ws_mutant_verify_identity "$2/cerulion-ros2-migrate-clang-mutant-unbound" "run it (unbound)"; then echo "UNBOUND-ACCEPTED"; fi
ws_mutants_restore >/dev/null 2>&1; true' \
    _ "$PWD/matrix_lib.sh" "$W31G" 2>&1) \
    || fail "31(g): the verify child failed before its oracle: $OUT31G"
case "$OUT31G" in
    *SWAPPED-ACCEPTED*) fail "31(g): ws_mutant_verify_identity ACCEPTED a swapped artifact — \
run_mutant would execute an object that is not the one it built: $OUT31G" ;;
esac
case "$OUT31G" in
    *UNBOUND-ACCEPTED*) fail "31(g): ws_mutant_verify_identity ACCEPTED an entry with no \
bound identity: $OUT31G" ;;
esac
case "$OUT31G" in
    *"NOT the artifact this run built"*) ;;
    *) fail "31(g): the swapped-artifact refusal does not name its cause: $OUT31G" ;;
esac

# (h) the wiring in run_matrix.sh: no removal by bare pathname remains; the
# identity is bound AFTER the mutant is built (before it, the identity would
# be the PREVIOUS run's binary, or nothing), verified BEFORE the mutant is
# run, and a refusal at either the verify or the final removal fails the
# mutant — inside `if ! run_mutant`, set -e is off, so the `|| return` is
# what carries the status.
W31_CODE=$(join_logical_lines run_matrix.sh)
# The CLASS, not one spelling: any `rm` command word on a line that names a
# mutant path — `rm -f "$mut"`, `rm "${mut}"`, `rm -f -- "$mut"`, `rm -rf` —
# is a removal by pathname. A grep for a single spelling was
# measured to pass with `rm -f "${mut}"` back in the failure block. The view
# is the comment-stripped join, so a comment naming `rm` cannot fire it
# (measured); a STRING carrying an `rm ... mut` phrase would, and that is
# the right direction for a gate — say it differently, not silently.
W31_BARE=$(printf '%s\n' "$W31_CODE" \
    | grep -E '(^|[;&|{(][[:space:]]*|[[:space:]])rm([[:space:]]|$)' | grep -c 'mut' || true)
[ "$W31_BARE" -eq 0 ] \
    || fail "31(h): run_matrix.sh still removes a mutant by bare pathname at $W31_BARE \
site(s) — a removal outside ws_mutant_remove deletes whatever the name denotes"
W31_BUILD=$(printf '%s\n' "$W31_CODE" | grep -n 'build\.sh' | grep -- '--mutant' \
    | head -1 | cut -d: -f1 || true)
# shellcheck disable=SC2016
W31_BIND=$(printf '%s\n' "$W31_CODE" | grep -n 'ws_mutant_bind_identity "$mut"' \
    | cut -d: -f1 || true)
# shellcheck disable=SC2016
W31_VERIFY=$(printf '%s\n' "$W31_CODE" | grep -n 'ws_mutant_verify_identity "$mut"' \
    | cut -d: -f1 || true)
# shellcheck disable=SC2016
W31_RUN=$(printf '%s\n' "$W31_CODE" | grep -n 'ws_capture_result "$rdir/mut.json"' \
    | head -1 | cut -d: -f1 || true)
[ -n "$W31_BUILD" ] && [ -n "$W31_RUN" ] \
    || fail "31(h): could not locate the mutant build and run sites in run_matrix.sh"
[ "$(printf '%s\n' "$W31_BIND" | grep -c . || true)" -eq 1 ] \
    || fail "31(h): expected exactly one ws_mutant_bind_identity \"\$mut\" site, found: \
'$W31_BIND'"
[ "$(printf '%s\n' "$W31_VERIFY" | grep -c . || true)" -eq 1 ] \
    || fail "31(h): expected exactly one ws_mutant_verify_identity \"\$mut\" site, found: \
'$W31_VERIFY'"
[ "$W31_BIND" -gt "$W31_BUILD" ] \
    || fail "31(h): the identity is bound (line $W31_BIND) BEFORE the mutant is built (line \
$W31_BUILD) — it would be the previous binary's identity, or none"
[ "$W31_VERIFY" -gt "$W31_BIND" ] && [ "$W31_VERIFY" -lt "$W31_RUN" ] \
    || fail "31(h): the verify (line $W31_VERIFY) does not sit between the bind (line \
$W31_BIND) and the run (line $W31_RUN)"
# shellcheck disable=SC2016
printf '%s\n' "$W31_CODE" | grep -q 'ws_mutant_verify_identity "$mut" .*|| return 2' \
    || fail "31(h): run_matrix.sh drops the verify's status — a swapped artifact would be RUN"
# shellcheck disable=SC2016
printf '%s\n' "$W31_CODE" | grep -q 'ws_mutant_remove "$mut" || return 2' \
    || fail "31(h): a refused final removal does not fail the mutant — the verdict was \
taken over an object something else has since replaced"
# The build-bind gap: the mutant is built INTO the run-private
# root and the registered path is derived under it, so the bound object was
# never at a name another run could write.
# shellcheck disable=SC2016
printf '%s\n' "$W31_CODE" | grep 'build\.sh' | grep -- '--mutant' \
    | grep -q -- '--out-dir "$WS_MUTANT_ROOT"' \
    || fail "31(h): the mutant build site does not build into the run-private root — a \
mutant at the shared name is exposed to the build-bind gap"
# shellcheck disable=SC2016
printf '%s\n' "$W31_CODE" | grep -q 'mut="$WS_MUTANT_ROOT/cerulion-ros2-migrate-clang-mutant-$lower"' \
    || fail "31(h): the registered mutant path is not derived under the run-private root"
# A failed bind removes NOTHING (no identity ⇒ no removal) and fails the
# mutant. The block from the bind line to its closing brace is what is read:
# a removal of any spelling in it is the blind arm coming back.
# shellcheck disable=SC2016
W31_BINDBLOCK=$(printf '%s\n' "$W31_CODE" \
    | sed -n '/ws_mutant_bind_identity "$mut"/,/^    }/p')
printf '%s\n' "$W31_BINDBLOCK" | grep -q 'ws_mutant_bind_identity' \
    || fail "31(h): could not locate the bind block in run_matrix.sh"
if printf '%s\n' "$W31_BINDBLOCK" \
        | grep -qE 'ws_mutant_remove|(^|[;&|{(][[:space:]]*|[[:space:]])rm([[:space:]]|$)'; then
    fail "31(h): a FAILED bind is followed by a removal — an object with no identity is not \
this run's to delete: $W31_BINDBLOCK"
fi
printf '%s\n' "$W31_BINDBLOCK" | grep -q 'return 2' \
    || fail "31(h): a failed bind does not fail the mutant"
# The run lock precedes the leftover check (so a leftover root is provably
# a dead run's), and the private root is minted after the prover bind and
# before the first mutant build.
# shellcheck disable=SC2016
W31_LOCK=$(printf '%s\n' "$W31_CODE" | grep -n '^ws_run_lock "$HERE"' | head -1 | cut -d: -f1 || true)
# shellcheck disable=SC2016
W31_CLEAN=$(printf '%s\n' "$W31_CODE" | grep -n '^ws_mutants_require_clean "$HERE"' \
    | head -1 | cut -d: -f1 || true)
[ -n "$W31_LOCK" ] && [ -n "$W31_CLEAN" ] \
    || fail "31(h): could not locate the run lock and the leftover check in run_matrix.sh"
[ "$W31_LOCK" -lt "$W31_CLEAN" ] \
    || fail "31(h): the run lock (line $W31_LOCK) is taken AFTER the leftover check (line \
$W31_CLEAN) — a leftover root could be a live run's"
W31_ROOT=$(printf '%s\n' "$W31_CODE" | grep -n '^ws_mutant_root_create' | head -1 | cut -d: -f1 || true)
W31_PBIND=$(printf '%s\n' "$W31_CODE" | grep -n 'ws_mutants_bind ' | head -1 | cut -d: -f1 || true)
[ -n "$W31_ROOT" ] && [ -n "$W31_PBIND" ] \
    || fail "31(h): could not locate the private root mint and the prover bind in run_matrix.sh"
[ "$W31_ROOT" -gt "$W31_PBIND" ] && [ "$W31_ROOT" -lt "$W31_BUILD" ] \
    || fail "31(h): the private root (line $W31_ROOT) is not minted between the prover bind \
(line $W31_PBIND) and the first mutant build (line $W31_BUILD)"
# (i) a SYMLINK planted at a BOUND name. The object the name now denotes is
# the link, whose OWN identity (the key never follows) is not the bound one —
# so the verify, the removal and the sweep all refuse, and the link's target
# survives. A key that followed would read the target's identity through the
# link and accept: run_mutant would EXECUTE through the link, and the removal
# would delete only the link, leaving a prover with a proof compiled out
# behind. Measured on a draft with `-L` on both stat branches: both accepted.
W31I=$(w31_tool i)
# shellcheck disable=SC2016  # $1/$2/$M expand in the child shell
OUT31I=$(timeout 30 bash -c "$W31_PRELUDE"'
mv "$M" "$M.kept" || exit 9
ln -s "$M.kept" "$M" || exit 9
[ -L "$M" ] || { echo "PRECONDITION: no link at the name"; exit 9; }
'"$W31_CHANGED"'
if ws_mutant_verify_identity "$M" "run it (linked)"; then echo "VERIFY-ACCEPTED-LINK"; fi
if ws_mutant_remove "$M"; then echo "REMOVE-ACCEPTED-LINK"; fi
if [ -L "$M" ]; then echo "LINK-SURVIVED-REMOVE"; fi
ws_mutants_restore
if [ -L "$M" ]; then echo "LINK-SURVIVED-SWEEP"; fi
if [ "$(cat "$M.kept")" = "MUTANT-BYTES" ]; then echo "TARGET-INTACT"; fi
if ws_mutants_restore_failed; then echo LATCHED; else echo CLEAN; fi' \
    _ "$PWD/matrix_lib.sh" "$W31I" 2>&1) \
    || fail "31(i): the symlink child failed before its oracle: $OUT31I"
case "$OUT31I" in
    *VERIFY-ACCEPTED-LINK*) fail "31(i): ws_mutant_verify_identity ACCEPTED a symlink at a \
bound name — the key FOLLOWS the link, so run_mutant would execute through it: $OUT31I" ;;
esac
case "$OUT31I" in
    *REMOVE-ACCEPTED-LINK*) fail "31(i): ws_mutant_remove ACCEPTED a symlink at a bound \
name: $OUT31I" ;;
esac
case "$OUT31I" in
    *LINK-SURVIVED-REMOVE*) ;;
    *) fail "31(i): the link at a bound name was DELETED by ws_mutant_remove: $OUT31I" ;;
esac
case "$OUT31I" in
    *LINK-SURVIVED-SWEEP*) ;;
    *) fail "31(i): the link at a bound name was DELETED by the sweep: $OUT31I" ;;
esac
case "$OUT31I" in
    *TARGET-INTACT*) ;;
    *) fail "31(i): the link's TARGET was touched: $OUT31I" ;;
esac
case "$OUT31I" in
    *LATCHED*) ;;
    *) fail "31(i): a bound name carrying a link did not FAIL the run: $OUT31I" ;;
esac
# ...and the BIND gate: a link at a registered name never binds (a build does
# not produce one), and an unregistered path never binds (an identity with no
# registration would protect nothing).
W31I2=$(w31_tool i2)
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
OUT31I2=$(timeout 30 bash -c '. "$1" || exit 9
ws_mutants_bind "$2" "$2/cerulion-ros2-migrate-clang" || exit 9
M="$2/cerulion-ros2-migrate-clang-mutant-x"
ws_mutant_register "$M" || exit 9
printf "TARGET\n" > "$2/target" || exit 9
ln -s "$2/target" "$M" || exit 9
if ws_mutant_bind_identity "$M"; then echo "BOUND-A-LINK"; fi
printf "X\n" > "$2/cerulion-ros2-migrate-clang-mutant-unregistered" || exit 9
if ws_mutant_bind_identity "$2/cerulion-ros2-migrate-clang-mutant-unregistered"; then echo "BOUND-UNREGISTERED"; fi
ws_mutants_restore >/dev/null 2>&1; true' \
    _ "$PWD/matrix_lib.sh" "$W31I2" 2>&1) \
    || fail "31(i): the bind-gate child failed before its oracle: $OUT31I2"
case "$OUT31I2" in
    *BOUND-A-LINK*) fail "31(i): ws_mutant_bind_identity BOUND a symlink: $OUT31I2" ;;
esac
case "$OUT31I2" in
    *"not a regular file"*) ;;
    *) fail "31(i): the link refusal at the bind does not name its cause: $OUT31I2" ;;
esac
case "$OUT31I2" in
    *BOUND-UNREGISTERED*) fail "31(i): ws_mutant_bind_identity BOUND an unregistered path: \
$OUT31I2" ;;
esac
case "$OUT31I2" in
    *"is not registered by this"*) ;;
    *) fail "31(i): the unregistered refusal at the bind does not name its cause: $OUT31I2" ;;
esac
rm -f "$W31I2/cerulion-ros2-migrate-clang-mutant-unregistered"

# (j) a name's SECOND life begins unbound. Registering a name whose first
# life was bound, then interrupting the rebuild (a half-written object, never
# identity-bound), is refused for the TRUE reason — "never bound to an
# identity" — by the seam and by the sweep, never as a SWAP of the first
# life's artifact ("NOT the artifact this run built"), which would send the
# operator looking for an intruder when a build simply never completed.
# Measured on a draft without the clearing register: the swap message.
W31J=$(w31_tool j)
# shellcheck disable=SC2016  # $1/$2/$M expand in the child shell
OUT31J=$(timeout 30 bash -c "$W31_PRELUDE"'
ws_mutant_register "$M" || exit 9
rm -f "$M" || exit 9
printf "HALF-WRITTEN-REBUILD\n" > "$M" || exit 9
if ws_mutant_remove "$M"; then echo "SECOND-LIFE-REMOVED"; fi
if [ -e "$M" ]; then echo "SECOND-LIFE-SURVIVED-REMOVE"; fi
ws_mutants_restore
if [ -e "$M" ]; then echo "SECOND-LIFE-SURVIVED-SWEEP"; fi
if ws_mutants_restore_failed; then echo LATCHED; else echo CLEAN; fi' \
    _ "$PWD/matrix_lib.sh" "$W31J" 2>&1) \
    || fail "31(j): the second-life child failed before its oracle: $OUT31J"
case "$OUT31J" in
    *SECOND-LIFE-REMOVED*) fail "31(j): ws_mutant_remove REMOVED the half-written product of \
an interrupted REBUILD — an entry with no identity: $OUT31J" ;;
esac
case "$OUT31J" in
    *SECOND-LIFE-SURVIVED-REMOVE*) ;;
    *) fail "31(j): the half-written rebuild was DELETED by the seam: $OUT31J" ;;
esac
case "$OUT31J" in
    *SECOND-LIFE-SURVIVED-SWEEP*) ;;
    *) fail "31(j): the half-written rebuild was DELETED by the sweep: $OUT31J" ;;
esac
case "$OUT31J" in
    *LATCHED*) ;;
    *) fail "31(j): a half-written rebuild left in place did not FAIL the run: $OUT31J" ;;
esac
case "$OUT31J" in
    *"NOT the artifact this run built"*) fail "31(j): an interrupted REBUILD was refused as a \
SWAP of the first life's artifact — the first life's identity survived the re-registration, \
and the operator is sent looking for an intruder: $OUT31J" ;;
esac
W31J_NAMED=$(printf '%s\n' "$OUT31J" | grep -c "never bound to an identity" || true)
[ "$W31J_NAMED" -eq 2 ] \
    || fail "31(j): expected the never-bound refusal from the seam AND the sweep (2), got \
$W31J_NAMED: $OUT31J"

# (k) a MALFORMED identity rendering is refused fail-closed at the CONSUMERS
# — the verify and both removals read the name through this host's stat —
# and reported as "cannot be identified", never as a swap (an operator told
# "something replaced it" would hunt an intruder when their stat dialect is
# the problem). The bind does not read the name at all (its
# identity comes from the stamped descriptor), so the stub `stat` is
# inserted AFTER the bind and asserted to be the stat in force. Control: a
# stub rendering exactly the bound identity passes the verify and the
# removal.
W31K=$(w31_tool k)
mkdir -p "$W31K/stub-nofrac" "$W31K/stub-spaced" "$W31K/stub-multiline" "$W31K/stub-control"
printf '#!/bin/sh\necho 1:2:3:4\n' > "$W31K/stub-nofrac/stat"
printf '#!/bin/sh\necho "1:2:3:4.5 6"\n' > "$W31K/stub-spaced/stat"
printf '#!/bin/sh\nprintf "1:2:3:4.5\\n6\\n"\n' > "$W31K/stub-multiline/stat"
chmod 755 "$W31K"/stub-*/stat
# shellcheck disable=SC2016  # $1/$2/$3/$M expand in the child shell
OUT31K=$(timeout 30 bash -c "$W31_PRELUDE"'
printf "#!/bin/sh\necho %s\n" "$(ws_mutant_identity_of "$M")" > "$3/stat" || exit 9
chmod 755 "$3/stat" || exit 9
W31K_PATH0=$PATH
PATH="$3:$PATH"
[ "$(command -v stat)" = "$3/stat" ] || { echo "PRECONDITION: the stub stat is not in force"; exit 9; }
ws_mutant_verify_identity "$M" "run it (control)" || { echo "CONTROL-VERIFY-REFUSED"; exit 9; }
ws_mutant_remove "$M" || { echo "CONTROL-REMOVE-REFUSED"; exit 9; }
[ ! -e "$M" ] || { echo "CONTROL-LEFT-A-MUTANT"; exit 9; }
PATH=$W31K_PATH0
ws_mutants_restore >/dev/null 2>&1; echo CONTROL-OK' \
    _ "$PWD/matrix_lib.sh" "$W31K" "$W31K/stub-control" 2>&1) \
    || fail "31(k): the control-stub child failed before its oracle: $OUT31K"
case "$OUT31K" in
    *CONTROL-OK*) ;;
    *) fail "31(k): the CONTROL stub (rendering the bound identity) did not pass the verify and \
the removal — the malformed arms below could refuse for an apparatus reason: $OUT31K" ;;
esac
for W31K_STUB in nofrac spaced multiline; do
    # shellcheck disable=SC2016  # $1/$2/$3/$M expand in the child shell
    OUT31K=$(timeout 30 bash -c "$W31_PRELUDE"'
W31K_PATH0=$PATH
PATH="$3:$PATH"
[ "$(command -v stat)" = "$3/stat" ] || { echo "PRECONDITION: the stub stat is not in force"; exit 9; }
if ws_mutant_verify_identity "$M" "run it (malformed)"; then echo "VERIFY-PASSED-MALFORMED"; fi
if ws_mutant_remove "$M"; then echo "REMOVE-ACCEPTED-MALFORMED"; fi
[ -f "$M" ] || { echo "MALFORMED-DELETED"; exit 9; }
ws_mutants_restore
if ws_mutants_restore_failed; then echo LATCHED; else echo CLEAN; fi
PATH=$W31K_PATH0
rm -f "$M"   # the fixture cleans its own mutant: under the real stat the sweep already ran' \
        _ "$PWD/matrix_lib.sh" "$W31K" "$W31K/stub-$W31K_STUB" 2>&1) \
        || fail "31(k): the malformed-identity child ($W31K_STUB) failed before its oracle: $OUT31K"
    case "$OUT31K" in
        *VERIFY-PASSED-MALFORMED*) fail "31(k): ws_mutant_verify_identity PASSED under a stat \
rendering a malformed key ($W31K_STUB) — a weaker or unmatchable key was accepted: $OUT31K" ;;
    esac
    case "$OUT31K" in
        *REMOVE-ACCEPTED-MALFORMED*) fail "31(k): ws_mutant_remove REMOVED under a stat rendering \
a malformed key ($W31K_STUB): $OUT31K" ;;
    esac
    case "$OUT31K" in
        *"NOT the artifact this run built"*) fail "31(k): a malformed rendering ($W31K_STUB) was \
reported as a SWAP — the operator is sent hunting an intruder for a stat dialect: $OUT31K" ;;
    esac
    W31K_NAMED=$(printf '%s\n' "$OUT31K" | grep -c "cannot be identified" || true)
    [ "$W31K_NAMED" -eq 3 ] \
        || fail "31(k): expected the cannot-be-identified refusal from the verify, the removal AND \
the sweep (3) under a malformed rendering ($W31K_STUB), got $W31K_NAMED: $OUT31K"
    case "$OUT31K" in
        *LATCHED*) ;;
        *) fail "31(k): a malformed rendering ($W31K_STUB) left the artifact in place WITHOUT \
failing the run: $OUT31K" ;;
    esac
done

# (l) a bound artifact that VANISHED is said out loud — the mirror image of
# the swap the seam refuses — by the removal (rc 0: nothing outlives the
# matrix) and by the sweep (no latch: the directory is clean).
W31L=$(w31_tool l)
# shellcheck disable=SC2016  # $1/$2/$M expand in the child shell
OUT31L=$(timeout 30 bash -c "$W31_PRELUDE"'
rm -f "$M" || exit 9
ws_mutant_remove "$M" || { echo "VANISHED-REFUSED"; exit 9; }
ws_mutant_register "$M" || exit 9
printf "MUTANT-BYTES\n" > "$M" || exit 9
ws_mutant_bind_identity "$M" || exit 9
rm -f "$M" || exit 9
ws_mutants_restore
if ws_mutants_restore_failed; then echo LATCHED; else echo CLEAN; fi' \
    _ "$PWD/matrix_lib.sh" "$W31L" 2>&1) \
    || fail "31(l): the vanished-artifact child failed before its oracle: $OUT31L"
W31L_WARNINGS=$(printf '%s\n' "$OUT31L" | grep -c "VANISHED" || true)
[ "$W31L_WARNINGS" -eq 2 ] \
    || fail "31(l): expected the VANISHED warning from the removal AND from the sweep (2), \
got $W31L_WARNINGS: $OUT31L"
case "$OUT31L" in
    *CLEAN*) ;;
    *) fail "31(l): a vanished artifact latched a restore failure — nothing outlived the \
matrix: $OUT31L" ;;
esac

# (m) a removal that FAILS carries rm's own reason, and the sweep latches.
# Driven with a stub `rm` on PATH that refuses every mutant path (and execs
# the real rm for anything else, so the pristine copy is still cleaned up):
# uid-independent, where a permission fixture would pass as root — the
# container's uid — and a directory at the name
# is refused BEFORE any rm as never bound ((m2) below). Inserted after
# the bind, asserted to be the rm in force; the bound file must SURVIVE it
# (the stub removes nothing) — the anti-vacuity half.
W31M=$(w31_tool m)
mkdir -p "$W31M/stub-rm"
printf '#!/bin/sh\ncase "$*" in *-mutant-*) echo "rm: cannot remove: Operation not permitted" >&2; exit 1 ;; esac\nexec /bin/rm "$@"\n' \
    > "$W31M/stub-rm/rm"
chmod 755 "$W31M/stub-rm/rm"
# shellcheck disable=SC2016  # $1/$2/$3/$M expand in the child shell
OUT31M=$(timeout 30 bash -c "$W31_PRELUDE"'
W31M_PATH0=$PATH
PATH="$3:$PATH"
[ "$(command -v rm)" = "$3/rm" ] || { echo "PRECONDITION: the stub rm is not in force"; exit 9; }
if ws_mutant_remove "$M"; then echo "FAILED-RM-ACCEPTED"; fi
[ -f "$M" ] || { echo "STUB-REMOVED-IT"; exit 9; }
ws_mutants_restore
if ws_mutants_restore_failed; then echo LATCHED; else echo CLEAN; fi
[ -f "$M" ] || { echo "STUB-REMOVED-IT-IN-SWEEP"; exit 9; }
PATH=$W31M_PATH0' \
    _ "$PWD/matrix_lib.sh" "$W31M" "$W31M/stub-rm" 2>&1) \
    || fail "31(m): the failed-removal child failed before its oracle: $OUT31M"
case "$OUT31M" in
    *FAILED-RM-ACCEPTED*) fail "31(m): ws_mutant_remove reported success over an rm that \
refused: $OUT31M" ;;
esac
case "$OUT31M" in
    *"could NOT be removed: "*"Operation not permitted"*) ;;
    *) fail "31(m): the failed removal does not carry rm's own reason: $OUT31M" ;;
esac
case "$OUT31M" in
    *LATCHED*) ;;
    *) fail "31(m): a registered name the sweep could not clear did not FAIL the run: \
$OUT31M" ;;
esac
rm -f "$W31M/cerulion-ros2-migrate-clang-mutant-x"
# (m2) a DIRECTORY at a registered, never-bound name is refused as NEVER
# BOUND — the identity gate precedes the rm, so rm's own directory refusal
# is never reached — the directory survives, and the sweep latches.
W31M2=$(w31_tool m2)
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
OUT31M2=$(timeout 30 bash -c '. "$1" || exit 9
ws_mutants_bind "$2" "$2/cerulion-ros2-migrate-clang" || exit 9
M="$2/cerulion-ros2-migrate-clang-mutant-x"
ws_mutant_register "$M" || exit 9
mkdir "$M" || exit 9
if ws_mutant_remove "$M"; then echo "DIRECTORY-REMOVE-ACCEPTED"; fi
[ -d "$M" ] || { echo "DIRECTORY-GONE"; exit 9; }
ws_mutants_restore
if ws_mutants_restore_failed; then echo LATCHED; else echo CLEAN; fi
rmdir "$M" || exit 9' \
    _ "$PWD/matrix_lib.sh" "$W31M2" 2>&1) \
    || fail "31(m2): the directory child failed before its oracle: $OUT31M2"
case "$OUT31M2" in
    *DIRECTORY-REMOVE-ACCEPTED*) fail "31(m2): ws_mutant_remove reported success over a \
directory at a never-bound name: $OUT31M2" ;;
esac
case "$OUT31M2" in
    *"never bound to an identity"*) ;;
    *) fail "31(m2): a directory at a never-bound name was not refused as never bound — the \
identity gate no longer precedes the rm: $OUT31M2" ;;
esac
case "$OUT31M2" in
    *LATCHED*) ;;
    *) fail "31(m2): a never-bound directory left in place did not FAIL the run: $OUT31M2" ;;
esac

# (n) the COALESCED-CLOCK model. Linux file timestamps come from
# the coarse clock (jiffies granularity), so a same-inode, same-size rewrite
# landing in the same tick as the build keeps dev:inode:size:mtime — the
# exact shape (d) refuses only because this desk's clock is fine. The bound
# identity therefore carries a RUN-OWNED stamp, applied at the bind, that no
# clock write reproduces. Modelled without a coarse clock: the peer rewrites
# the bytes in place, then the mtime is put back to what the BUILD's own
# write left (`W31_RAW_PRE`, read before the bind) — what a coalesced clock
# would have stored. The removal and the sweep must refuse. Without the
# stamp (measured on the unstamped key) the bound tuple IS that pre-bind
# reading, the model matches it, and the peer's artifact is deleted.
W31N=$(w31_tool n)
# shellcheck disable=SC2016  # $1/$2/$M expand in the child shell
OUT31N=$(timeout 30 bash -c "$W31_PRELUDE"'
if [ "$W31_RAW_PRE" = "$W31_RAW0" ]; then echo "STAMP-ABSENT"; fi
printf "MUTANT-BYTEZ\n" > "$M" || exit 9
python3 - "$M" "$W31_RAW_PRE" <<"PYEOF31N"
import os, sys
sec, frac = sys.argv[2].split(":")[3].split(".")
ns = int(sec) * 10**9 + int((frac + "000000000")[:9])
os.utime(sys.argv[1], ns=(ns, ns))
PYEOF31N
[ "$(w31_raw "$M")" = "$W31_RAW_PRE" ] || { echo "PRECONDITION: the coalesced-clock model did not land: $(w31_raw "$M") vs $W31_RAW_PRE"; exit 9; }
if ws_mutant_remove "$M"; then echo "REMOVE-ACCEPTED"; fi
if [ -e "$M" ]; then echo SURVIVED-REMOVE; fi
ws_mutants_restore
if [ -e "$M" ]; then echo SURVIVED-SWEEP; fi
if ws_mutants_restore_failed; then echo LATCHED; else echo CLEAN; fi' \
    _ "$PWD/matrix_lib.sh" "$W31N" 2>&1) \
    || fail "31(n): the coalesced-clock child failed before its oracle: $OUT31N"
case "$OUT31N" in
    *STAMP-ABSENT*) fail "31(n): the bind did not move the artifact's mtime — the identity \
carries the clock's value, and a coalesced clock keeps it across a rewrite: $OUT31N" ;;
esac
case "$OUT31N" in
    *REMOVE-ACCEPTED*) fail "31(n): a same-inode, same-size rewrite whose mtime a coalesced \
clock put back on the build's tick was accepted as this run's artifact: $OUT31N" ;;
esac
case "$OUT31N" in
    *SURVIVED-REMOVE*) ;;
    *) fail "31(n): the rewritten object was DELETED by the removal: $OUT31N" ;;
esac
case "$OUT31N" in
    *SURVIVED-SWEEP*) ;;
    *) fail "31(n): the rewritten object was DELETED by the sweep: $OUT31N" ;;
esac
case "$OUT31N" in
    *LATCHED*) ;;
    *) fail "31(n): a coalesced-clock rewrite did not FAIL the run: $OUT31N" ;;
esac

# (o) a write landing AFTER the stamp. The identity bound is the
# stamped DESCRIPTOR's (python fstat), so nothing between the stamp and the
# identity read happens by name — and a write that lands after it is caught
# at the next check by name: the verify refuses, the removal refuses, the
# sweep latches. Modelled by wrapping the real stamp in one that rewrites
# the file after stamping and passes the identity through.
W31O=$(w31_tool o)
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
OUT31O=$(timeout 30 bash -c '. "$1" || exit 9
ws_mutants_bind "$2" "$2/cerulion-ros2-migrate-clang" || exit 9
M="$2/cerulion-ros2-migrate-clang-mutant-x"
ws_mutant_register "$M" || exit 9
printf "MUTANT-BYTES\n" > "$M" || exit 9
eval "$(declare -f ws_stamp_artifact | sed "1s/ws_stamp_artifact/w31_real_stamp/")"
ws_stamp_artifact() { local id; id=$(w31_real_stamp "$1") || return $?; printf "MUTANT-BYTEZ\n" > "$1" || return 9; printf "%s\n" "$id"; }
ws_mutant_bind_identity "$M" || { echo "BIND-REFUSED"; exit 9; }
if ws_mutant_verify_identity "$M" "run it"; then echo "VERIFY-PASSED-AFTER-WRITE"; fi
if ws_mutant_remove "$M"; then echo "REMOVE-ACCEPTED-AFTER-WRITE"; fi
[ -f "$M" ] || { echo "DELETED-AFTER-WRITE"; exit 9; }
ws_mutants_restore
if ws_mutants_restore_failed; then echo LATCHED; else echo CLEAN; fi' \
    _ "$PWD/matrix_lib.sh" "$W31O" 2>&1) \
    || fail "31(o): the write-after-stamp child failed before its oracle: $OUT31O"
case "$OUT31O" in
    *VERIFY-PASSED-AFTER-WRITE*|*REMOVE-ACCEPTED-AFTER-WRITE*) fail "31(o): an artifact written \
AFTER the stamp was accepted as this run's — the identity did not come from the stamped \
descriptor, or the check by name is gone: $OUT31O" ;;
esac
case "$OUT31O" in
    *"NOT the artifact this run built"*) ;;
    *) fail "31(o): the write after the stamp was not refused as a swap by name: $OUT31O" ;;
esac
case "$OUT31O" in
    *LATCHED*) ;;
    *) fail "31(o): an artifact written after the stamp did not FAIL the run: $OUT31O" ;;
esac

# (r) a LINK swapped in at the registered name AFTER the by-name
# gate (modelled by disabling the gate) is refused by the KERNEL at the
# open (O_NOFOLLOW), never stamped, never bound; the link's target — a
# peer's file — is untouched and keeps its own mtime.
W31R=$(w31_tool r)
# shellcheck disable=SC2016  # $1/$2/$M expand in the child shell
OUT31R=$(timeout 30 bash -c '. "$1" || exit 9
w31_raw() { stat -c "%d:%i:%s:%.9Y" "$1" 2>/dev/null || stat -f "%d:%i:%z:%Fm" "$1"; }
ws_mutants_bind "$2" "$2/cerulion-ros2-migrate-clang" || exit 9
M="$2/cerulion-ros2-migrate-clang-mutant-x"
ws_mutant_register "$M" || exit 9
printf "PEER\n" > "$2/peer-target" || exit 9
ln -s "$2/peer-target" "$M" || exit 9
[ -L "$M" ] || exit 9
ws_bind_precheck() { :; }
if ws_mutant_bind_identity "$M"; then echo "BOUND-THROUGH-A-LINK"; fi
[ "$(cat "$2/peer-target")" = "PEER" ] || echo "TARGET-CHANGED"
case "$(w31_raw "$2/peer-target")" in *:"$WS_MUTANT_STAMP_MTIME") echo "TARGET-STAMPED" ;; esac
ws_mutants_restore >/dev/null 2>&1; true' \
    _ "$PWD/matrix_lib.sh" "$W31R" 2>&1) \
    || fail "31(r): the swapped-link child failed before its oracle: $OUT31R"
case "$OUT31R" in
    *BOUND-THROUGH-A-LINK*) fail "31(r): the bind BOUND through a link swapped in after the \
by-name gate — the open follows links: $OUT31R" ;;
esac
case "$OUT31R" in
    *TARGET-STAMPED*|*TARGET-CHANGED*) fail "31(r): the link's target — a peer's file — was \
stamped or changed through the link: $OUT31R" ;;
esac
case "$OUT31R" in
    *"without following a link"*) ;;
    *) fail "31(r): the refusal does not name its cause: $OUT31R" ;;
esac
[ -L "$W31R/cerulion-ros2-migrate-clang-mutant-x" ] \
    || fail "31(r): the never-bound link was REMOVED by the sweep"
rm -f "$W31R/cerulion-ros2-migrate-clang-mutant-x"

# (p) one run per tool directory: the run lock refuses a second run BY NAME
# while a holder lives, releases when the holder exits (however it exits —
# the holder here is killed), creates NO lock file (the lock is
# on the directory's own descriptor), refuses a link or a non-directory at
# the path by name, and — the key pin — refuses a link swapped in
# AFTER the by-name gate (modelled by disabling the gate): the object is
# verified after the open, through the descriptor, so two runs can never
# flock two different objects.
W31P=$(w31_tool p)
mkdir -p "$T/w31p"
# shellcheck disable=SC2016  # $1/$2/$3 expand in the child shell
timeout 60 bash -c '. "$1" || exit 9
ws_run_lock "$2" || exit 9
printf "held\n" > "$3/held"
exec sleep 30' _ "$PWD/matrix_lib.sh" "$W31P" "$T/w31p" &
P31=$!
for _ in $(seq 1 100); do
    [ -s "$T/w31p/held" ] && break
    sleep 0.1
done
[ -s "$T/w31p/held" ] || fail "31(p): the holder never took the run lock (apparatus broken)"
[ ! -e "$W31P/.run_matrix.lock" ] \
    || fail "31(p): the run lock created a lock FILE — a name a peer can swap for a link between \
the check and the open"
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
OUT31P=$(timeout 30 bash -c '. "$1" || exit 9
if ws_run_lock "$2"; then echo "SECOND-ACQUIRED"; fi' \
    _ "$PWD/matrix_lib.sh" "$W31P" 2>&1) \
    || fail "31(p): the second-run child failed before its oracle: $OUT31P"
case "$OUT31P" in
    *SECOND-ACQUIRED*) fail "31(p): a second run TOOK the run lock while another held it: \
$OUT31P" ;;
esac
case "$OUT31P" in
    *"another run_matrix.sh is live in"*) ;;
    *) fail "31(p): the refusal does not name its cause: $OUT31P" ;;
esac
kill "$P31" 2>/dev/null || true
wait "$P31" 2>/dev/null || true
W31P_THIRD=""
for _ in $(seq 1 50); do
    # shellcheck disable=SC2016  # $1/$2 expand in the child shell
    W31P_THIRD=$(timeout 30 bash -c '. "$1" || exit 9
ws_run_lock "$2" || exit 75
echo THIRD-ACQUIRED' _ "$PWD/matrix_lib.sh" "$W31P" 2>&1) && break
    sleep 0.1
done
case "$W31P_THIRD" in
    *THIRD-ACQUIRED*) ;;
    *) fail "31(p): after the holder was killed the run lock was still held — it does not \
release with its holder: $W31P_THIRD" ;;
esac
# A link at the directory path, and a plain file there, are refused BY NAME.
W31P2=$(w31_tool p2)
ln -s "$W31P2" "$T/w31p2-link"
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
if OUT31P2=$(timeout 30 bash -c '. "$1" || exit 9; ws_run_lock "$2"' \
        _ "$PWD/matrix_lib.sh" "$T/w31p2-link" 2>&1); then
    fail "31(p): the run lock locked THROUGH a link at the directory path"
fi
case "$OUT31P2" in
    *"is not a directory"*) ;;
    *) fail "31(p): the link refusal does not name its cause: $OUT31P2" ;;
esac
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
if OUT31P2=$(timeout 30 bash -c '. "$1" || exit 9; ws_run_lock "$2"' \
        _ "$PWD/matrix_lib.sh" "$W31P2/cerulion-ros2-migrate-clang" 2>&1); then
    fail "31(p): the run lock locked a plain FILE named as the tool directory"
fi
# THE key pin: the by-name gate is disabled — the swap lands after it —
# and the object opened must still be refused, because it is verified after
# the open through the descriptor (fstat a directory; lstat of the name not
# a link and the same dev:inode). The link resolves to a real directory, so
# a lock that trusted the open would ACQUIRE — on the wrong object.
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
OUT31P3=$(timeout 30 bash -c '. "$1" || exit 9
ws_run_lock_precheck() { :; }
if ws_run_lock "$2"; then echo "LOCKED-THROUGH-A-LINK"; fi
[ -z "$WS_RUN_LOCK_FD" ] || echo "LOCK-FD-KEPT"' \
    _ "$PWD/matrix_lib.sh" "$T/w31p2-link" 2>&1) \
    || fail "31(p): the swap-in-the-gap child failed before its oracle: $OUT31P3"
case "$OUT31P3" in
    *LOCKED-THROUGH-A-LINK*|*LOCK-FD-KEPT*) fail "31(p): a link swapped in after the by-name \
gate was LOCKED THROUGH — the object is not verified after the open, so two runs can flock \
two different objects: $OUT31P3" ;;
esac
case "$OUT31P3" in
    *"is not the directory that name denotes"*) ;;
    *) fail "31(p): the post-open refusal does not name its cause: $OUT31P3" ;;
esac

# (q) the run-private mutant root: refused without a bind, minted under the
# tool directory as a 0700 directory, refused twice, not counted as a
# leftover by this run's own leftover check, swept with the run (a bound
# mutant inside is removed, then the root), retained + named + latched when
# something it holds could not be removed, and refused BY NAME at the next
# run's startup when left behind — empty or not.
W31Q=$(w31_tool q)
# shellcheck disable=SC2016  # $1 expands in the child shell
if OUT31Q=$(timeout 30 bash -c '. "$1" || exit 9; ws_mutant_root_create' \
        _ "$PWD/matrix_lib.sh" 2>&1); then
    fail "31(q): ws_mutant_root_create minted a root with no production prover bound"
fi
case "$OUT31Q" in
    *"no production prover bound"*) ;;
    *) fail "31(q): the no-bind refusal does not name its cause: $OUT31Q" ;;
esac
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
OUT31Q=$(timeout 30 bash -c '. "$1" || exit 9
ws_mutants_bind "$2" "$2/cerulion-ros2-migrate-clang" || exit 9
ws_mutant_root_create || exit 9
[ -n "$WS_MUTANT_ROOT" ] || { echo "NO-ROOT-VAR"; exit 9; }
case "$WS_MUTANT_ROOT" in "$2"/.mutants.*) ;; *) echo "ROOT-ELSEWHERE:$WS_MUTANT_ROOT"; exit 9 ;; esac
[ -d "$WS_MUTANT_ROOT" ] && [ ! -L "$WS_MUTANT_ROOT" ] || { echo "ROOT-NOT-A-DIRECTORY"; exit 9; }
[ "$(ws_object_mode "$WS_MUTANT_ROOT")" = 700 ] || { echo "ROOT-MODE:$(ws_object_mode "$WS_MUTANT_ROOT")"; exit 9; }
if ws_mutant_root_create; then echo "SECOND-ROOT-MINTED"; fi
ws_mutants_require_clean "$2" || { echo "OWN-ROOT-REFUSED"; exit 9; }
M="$WS_MUTANT_ROOT/cerulion-ros2-migrate-clang-mutant-x"
ws_mutant_register "$M" || exit 9
printf "MUTANT-BYTES\n" > "$M" || exit 9
ws_mutant_bind_identity "$M" || exit 9
R=$WS_MUTANT_ROOT
ws_mutants_restore
[ ! -e "$M" ] || echo MUTANT-SURVIVED
[ ! -e "$R" ] || echo ROOT-SURVIVED
if ws_mutants_restore_failed; then echo LATCHED; else echo CLEAN; fi' \
    _ "$PWD/matrix_lib.sh" "$W31Q" 2>&1) \
    || fail "31(q): the root-lifecycle child failed before its oracle: $OUT31Q"
case "$OUT31Q" in
    *SECOND-ROOT-MINTED*) fail "31(q): a second private root was minted over a live one — the \
first is orphaned with whatever it holds: $OUT31Q" ;;
esac
case "$OUT31Q" in
    *MUTANT-SURVIVED*|*ROOT-SURVIVED*) fail "31(q): the sweep left the bound mutant or the \
private root behind: $OUT31Q" ;;
esac
case "$OUT31Q" in
    *CLEAN*) ;;
    *) fail "31(q): a clean root lifecycle latched a restore failure: $OUT31Q" ;;
esac
W31Q3=$(w31_tool q3)
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
OUT31Q3=$(timeout 30 bash -c '. "$1" || exit 9
ws_mutants_bind "$2" "$2/cerulion-ros2-migrate-clang" || exit 9
ws_mutant_root_create || exit 9
M="$WS_MUTANT_ROOT/cerulion-ros2-migrate-clang-mutant-x"
ws_mutant_register "$M" || exit 9
printf "HALF-WRITTEN\n" > "$M" || exit 9
R=$WS_MUTANT_ROOT
ws_mutants_restore
[ -e "$M" ] && echo MUTANT-SURVIVED
[ -d "$R" ] && echo ROOT-SURVIVED
echo "ROOT=$R"
if ws_mutants_restore_failed; then echo LATCHED; else echo CLEAN; fi' \
    _ "$PWD/matrix_lib.sh" "$W31Q3" 2>&1) \
    || fail "31(q): the retained-root child failed before its oracle: $OUT31Q3"
case "$OUT31Q3" in
    *MUTANT-SURVIVED*) ;;
    *) fail "31(q): a never-bound mutant inside the private root was DELETED: $OUT31Q3" ;;
esac
case "$OUT31Q3" in
    *ROOT-SURVIVED*) ;;
    *) fail "31(q): a private root still holding a refused entry was REMOVED: $OUT31Q3" ;;
esac
case "$OUT31Q3" in
    *"private mutant root could NOT be removed"*) ;;
    *) fail "31(q): the retained root was not named: $OUT31Q3" ;;
esac
case "$OUT31Q3" in
    *LATCHED*) ;;
    *) fail "31(q): a retained private root did not FAIL the run: $OUT31Q3" ;;
esac
W31Q3_ROOT=$(printf '%s\n' "$OUT31Q3" | sed -n 's/^ROOT=//p' | head -1)
[ -n "$W31Q3_ROOT" ] && [ -d "$W31Q3_ROOT" ] \
    || fail "31(q): the retained-root child did not report its root (apparatus broken)"
# ...and the NEXT run refuses to start while it exists — by name, without
# entering or deleting it — as does the bind, which runs the same check.
if OUT31Q4=$(ws_mutants_require_clean "$W31Q3" 2>&1); then
    fail "31(q): a tool directory holding a leftover private mutant root was ACCEPTED"
fi
case "$OUT31Q4" in
    *"private mutant root(s) left by an earlier run"*"$W31Q3_ROOT"*) ;;
    *) fail "31(q): the leftover-root refusal does not name the root: $OUT31Q4" ;;
esac
[ -f "$W31Q3_ROOT/cerulion-ros2-migrate-clang-mutant-x" ] \
    || fail "31(q): the leftover-root refusal DELETED what the root holds"
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
if timeout 30 bash -c '. "$1" || exit 9; ws_mutants_bind "$2" "$2/cerulion-ros2-migrate-clang"' \
        _ "$PWD/matrix_lib.sh" "$W31Q3" >/dev/null 2>&1; then
    fail "31(q): the prover bind ACCEPTED a tool directory holding a leftover private root"
fi
W31Q5=$(w31_tool q5)
mkdir "$W31Q5/.mutants.empty"
if ws_mutants_require_clean "$W31Q5" >/dev/null 2>&1; then
    fail "31(q): an EMPTY leftover private root was accepted — a run that did not clean up \
must be named even when it left nothing inside"
fi
rm -rf "$W31Q3_ROOT" "$W31Q5/.mutants.empty"
# (q6) a mutant INSIDE the private root that this run never registered — a
# build site that skipped its registration, something planted in a
# directory only this run should write — is reported by the survivor sweep
# as not registered, never deleted, and the root it keeps occupied is
# retained, named and latched.
W31Q6=$(w31_tool q6)
# shellcheck disable=SC2016  # $1/$2 expand in the child shell
OUT31Q6=$(timeout 30 bash -c '. "$1" || exit 9
ws_mutants_bind "$2" "$2/cerulion-ros2-migrate-clang" || exit 9
ws_mutant_root_create || exit 9
printf "UNREGISTERED\n" > "$WS_MUTANT_ROOT/cerulion-ros2-migrate-clang-mutant-stray" || exit 9
R=$WS_MUTANT_ROOT
ws_mutants_restore
[ -f "$R/cerulion-ros2-migrate-clang-mutant-stray" ] && echo STRAY-SURVIVED
[ -d "$R" ] && echo ROOT-SURVIVED
echo "ROOT=$R"
if ws_mutants_restore_failed; then echo LATCHED; else echo CLEAN; fi' \
    _ "$PWD/matrix_lib.sh" "$W31Q6" 2>&1) \
    || fail "31(q6): the stray-in-root child failed before its oracle: $OUT31Q6"
case "$OUT31Q6" in
    *STRAY-SURVIVED*) ;;
    *) fail "31(q6): a mutant this run never registered was DELETED from its private root — \
report, never remove: $OUT31Q6" ;;
esac
case "$OUT31Q6" in
    *"survived this run and were not registered by it"*) ;;
    *) fail "31(q6): an unregistered mutant inside the private root survived in SILENCE: \
$OUT31Q6" ;;
esac
case "$OUT31Q6" in
    *ROOT-SURVIVED*"private mutant root could NOT be removed"*|*"private mutant root could NOT be removed"*ROOT-SURVIVED*) ;;
    *) fail "31(q6): the occupied root was removed, or not named: $OUT31Q6" ;;
esac
case "$OUT31Q6" in
    *LATCHED*) ;;
    *) fail "31(q6): an occupied private root left behind did not FAIL the run: $OUT31Q6" ;;
esac
W31Q6_ROOT=$(printf '%s\n' "$OUT31Q6" | sed -n 's/^ROOT=//p' | head -1)
[ -n "$W31Q6_ROOT" ] && rm -rf "$W31Q6_ROOT"

echo "artifact-bound registry OK (an untouched artifact is removed and unregistered, so a \
later occupant is reported as not-registered rather than latched as this run's failure; a \
replaced artifact — different bytes, a byte-identical copy under a new inode, or a same-size \
in-place rewrite — is refused by the removal and by the sweep, survives both, and latches a \
restore failure; an unregistered path is refused; a never-bound entry is refused, named and \
latched by both removal paths, never swept blind, and nothing at its name is not a failure; \
the pre-run verify passes only the bound artifact; a symlink at a bound name is refused by \
the verify, the removal and the sweep and never bound; a name's second life begins unbound \
and an interrupted rebuild is refused as never bound, not as a swap; a malformed rendering is \
refused fail-closed at the verify and both removals as cannot-be-identified, never as a swap, \
while a stub rendering the bound identity passes; a vanished bound artifact is reported by \
both removal paths; a removal rm refuses carries its reason and latches, and a directory at a \
never-bound name is refused before any rm; the bound identity carries a run-owned stamp taken \
through one O_NOFOLLOW descriptor whose identity equals the name's stat rendering, so a \
coalesced-clock rewrite is refused, a write after the stamp is refused at the next check, and \
a link swapped in after the by-name gate is refused by the kernel with its target untouched; \
one run per tool directory on the directory's own descriptor verified after the open — no lock \
file, a swapped link refused, a second run refused by name, released with its holder; every \
mutant is built into a run-private root that is swept with the run and \
refused at the next startup if left behind; run_matrix.sh locks first, mints the root after \
the prover bind, builds into it, binds after the build, verifies before the run, removes \
nothing after a failed bind, carries both statuses, and has no rm of any spelling on a \
mutant path)"

echo "run_matrix selftest: ALL OK ($ARMS)"
