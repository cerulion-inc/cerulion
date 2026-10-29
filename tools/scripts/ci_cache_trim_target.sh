#!/usr/bin/env bash
# ci_cache_trim_target.sh - drop the build units a job did not use from a
# cargo target directory before it is saved as a cache archive.
#
#   bash tools/scripts/ci_cache_trim_target.sh age  <target-dir>
#   bash tools/scripts/ci_cache_trim_target.sh trim <target-dir>
#   bash tools/scripts/ci_cache_trim_target.sh --self-test
#
# WHAT GROWS. A cache key rolls when a lockfile changes, and the new key's
# first run restores the previous archive through `restore-keys`. Cargo then
# writes the units whose hash changed (a version bump, a feature change, a new
# compiler) NEXT TO the old ones and never deletes the old ones, so every roll
# saves the previous generation plus the new one.
#
# HOW A UNIT IS JUDGED LIVE. Every cargo invocation reads the fingerprint files
# under `<profile>/.fingerprint/<name>-<hash>/` of every unit in its build graph,
# fresh or not. `age`, run right after the restore, sets the access time of
# every such file to 2000-01-01. A read then moves it to the present: Linux's
# `relatime` updates an access time that is older than the file's mtime, and
# APFS updates it on read. `trim`, run before the save, takes the hashes whose
# fingerprint directory holds a file accessed after 2001-01-01 as live, and in
# every profile directory deletes each `deps/`, `build/`, `examples/` and
# `.fingerprint/` entry whose name carries a 16-hex unit hash that is not live.
# Entries with no unit hash (the uplifted copies in the profile root,
# `.cargo-lock`) are left alone. `incremental/` is removed whole.
#
# EVERY PROFILE DIRECTORY, nested ones included: a directory holding a
# `.fingerprint/` anywhere under the target (for example the per-test-binary
# scaffold target dirs under `target/tmp/`, `CARGO_TARGET_TMPDIR`) is trimmed
# the same way. Under `target/tmp/` a top-level entry with no live unit at all
# is removed whole, which is also what clears scratch files tests leave there.
#
# WHEN IT DELETES NOTHING. `trim` exits 0 without deleting when
#   * the access-time probe fails: a file aged and then read on this filesystem
#     does not show a new access time (a `noatime` mount);
#   * the top profile directory `<target>/debug` holds no live unit (`age` did
#     not run, or nothing ran cargo);
#   * a profile directory has live units but no `deps/` entry carries one of
#     their hashes (the fingerprint-to-artifact naming no longer matches); that
#     profile is skipped and named.
# It prints the target size before and after, and the registry size beside it.
#
# Never fails the job on a trim problem: every path above prints and exits 0.
# Usage errors exit 2.

set -u

AGED_STAMP=200001010000      # what `age` writes as the access time
LIVE_AFTER_STAMP=200101010000 # an access time later than this is a read during the run

say() { printf 'ci_cache_trim: %s\n' "$*"; }

size_of() {
  if [ -e "$1" ]; then
    du -sk "$1" 2>/dev/null | awk '{ printf "%.2f GiB", $1 / 1048576 }'
  else
    printf 'absent'
  fi
}

# Every `.fingerprint` directory under the target, one per line.
fingerprint_dirs() {
  find "$1" -type d -name .fingerprint -prune -print 2>/dev/null
}

cmd_age() {
  local target="$1" fp n=0
  if [ ! -d "$target" ]; then
    say "age: $target does not exist (no cache restored); nothing to age"
    return 0
  fi
  while IFS= read -r fp; do
    [ -n "$fp" ] || continue
    find "$fp" -type f -exec touch -a -t "$AGED_STAMP" {} + 2>/dev/null || true
    n=$((n + 1))
  done <<EOF
$(fingerprint_dirs "$target")
EOF
  say "age: aged the fingerprint files of $n profile director(y/ies) under $target"
}

# Exit 0 when an aged file that is then read shows a new access time.
atime_probe() {
  local dir="$1" probe ref
  probe="$dir/.ci-cache-trim-probe.$$"
  ref="$dir/.ci-cache-trim-ref.$$"
  printf 'probe\n' > "$probe" || return 1
  touch -t "$LIVE_AFTER_STAMP" "$ref" || { rm -f "$probe"; return 1; }
  touch -a -t "$AGED_STAMP" "$probe"
  cat "$probe" > /dev/null
  local hit
  hit=$(find "$probe" -neweram "$ref" 2>/dev/null)
  rm -f "$probe" "$ref"
  [ -n "$hit" ]
}

# Print the live unit hashes of one `.fingerprint` directory, one per line.
live_hashes() {
  local fp="$1" ref="$2"
  find "$fp" -mindepth 2 -maxdepth 2 -type f -neweram "$ref" 2>/dev/null \
    | awk -F/ '{ print $(NF-1) }' \
    | sed -n 's/.*-\([0-9a-f]\{16\}\)$/\1/p' \
    | sort -u
}

# Print the entries of a profile directory that carry a unit hash, as
# "<hash> <path>" lines. A hash is `-<16 hex>` followed by `.` or the end.
hashed_entries() {
  local prof="$1" sub
  for sub in deps build examples .fingerprint; do
    [ -d "$prof/$sub" ] || continue
    find "$prof/$sub" -mindepth 1 -maxdepth 1 2>/dev/null
  done | awk -F/ '
    # Sixteen literal classes, not `{16}`: older mawk has no interval braces.
    BEGIN { h = "[0-9a-f]"; re = "-" h h h h h h h h h h h h h h h h "([.]|$)" }
    {
      name = $NF
      if (match(name, re)) {
        print substr(name, RSTART + 1, 16), $0
      }
    }'
}

trim_profile() {
  local prof="$1" ref="$2" work="$3"
  local live="$work/live" entries="$work/entries" stale="$work/stale"
  live_hashes "$prof/.fingerprint" "$ref" > "$live"
  hashed_entries "$prof" > "$entries"
  local n_live
  n_live=$(wc -l < "$live" | tr -d ' ')
  if [ "$n_live" -eq 0 ]; then
    case "$prof" in
      "$4"/tmp/*) ;;  # removed whole by the target/tmp pass below
      *) say "trim: kept $prof whole: no unit in it was read during this run" ;;
    esac
    return 0
  fi
  local matched
  matched=$(awk 'NR == FNR { live[$1] = 1; next }
                 index($2, "/deps/") && ($1 in live) { n++ }
                 END { print n + 0 }' "$live" "$entries")
  if [ "$matched" -eq 0 ]; then
    say "trim: SKIPPED $prof: $n_live live unit(s) but no deps/ entry carries one of their hashes"
    return 0
  fi
  awk 'NR == FNR { live[$1] = 1; next }
       !($1 in live) { sub(/^[^ ]+ /, ""); print }' "$live" "$entries" > "$stale"
  # Artifacts first, fingerprint directories last: a fingerprint whose outputs
  # are gone is rebuilt by cargo, so an interrupted trim leaves a usable tree.
  grep -v '/\.fingerprint/' "$stale" | while IFS= read -r p; do rm -rf "$p"; done
  grep '/\.fingerprint/' "$stale" | while IFS= read -r p; do rm -rf "$p"; done
  rm -rf "$prof/incremental"
  local n_stale
  n_stale=$(awk '{ print $1 }' "$entries" | sort -u \
            | awk 'NR == FNR { live[$1] = 1; next } !($1 in live) { n++ } END { print n + 0 }' "$live" -)
  say "trim: $prof: $n_live live unit(s), $n_stale stale unit hash(es) removed"
}

cmd_trim() {
  local target="$1"
  if [ ! -d "$target" ]; then
    say "trim: $target does not exist; nothing to trim"
    return 0
  fi
  say "trim: before: target $(size_of "$target"), cargo registry $(size_of "${CARGO_HOME:-$HOME/.cargo}/registry")"
  if ! atime_probe "$target"; then
    say "trim: SKIPPED: an aged file read on this filesystem kept its old access time (noatime?); deleting nothing"
    return 0
  fi
  local work ref
  work=$(mktemp -d "${TMPDIR:-/tmp}/ci-cache-trim.XXXXXX") || { say "trim: SKIPPED: no temp dir"; return 0; }
  ref="$work/ref"
  touch -t "$LIVE_AFTER_STAMP" "$ref"

  # The top profile first: no live unit there means `age` did not run or no
  # cargo command read this tree, and the whole trim stops.
  if [ -d "$target/debug/.fingerprint" ]; then
    if [ -z "$(live_hashes "$target/debug/.fingerprint" "$ref" | head -n 1)" ]; then
      say "trim: SKIPPED: $target/debug holds no unit read during this run; deleting nothing"
      rm -rf "$work"
      return 0
    fi
  fi

  local fp prof
  while IFS= read -r fp; do
    [ -n "$fp" ] || continue
    prof=$(dirname "$fp")
    trim_profile "$prof" "$ref" "$work" "$target"
  done <<EOF
$(fingerprint_dirs "$target")
EOF

  # `target/tmp` (CARGO_TARGET_TMPDIR): keep a top-level entry only when it
  # holds a profile directory with a live unit.
  if [ -d "$target/tmp" ]; then
    local entry keep
    for entry in "$target/tmp"/* "$target/tmp"/.[!.]*; do
      [ -e "$entry" ] || continue
      keep=0
      while IFS= read -r fp; do
        [ -n "$fp" ] || continue
        if [ -n "$(live_hashes "$fp" "$ref" | head -n 1)" ]; then keep=1; break; fi
      done <<EOF
$( [ -d "$entry" ] && fingerprint_dirs "$entry" )
EOF
      if [ "$keep" -eq 0 ]; then
        rm -rf "$entry"
        say "trim: removed $entry (no unit read during this run)"
      fi
    done
  fi
  rm -rf "$work"
  say "trim: after:  target $(size_of "$target")"
  return 0
}

# ---------------------------------------------------------------------------
# self-test: a synthetic target tree with a live and a stale generation.
# ---------------------------------------------------------------------------
self_test() {
  local base t fails=0
  base=$(mktemp -d "${TMPDIR:-/tmp}/ci-cache-trim-selftest.XXXXXX") || { echo "self-test: no temp dir" >&2; return 1; }
  t="$base/target"
  fail() { printf 'ci_cache_trim: self-test FAILED: %s\n' "$*" >&2; fails=$((fails + 1)); }

  mkunit() { # <profile> <name> <hash>
    local prof="$1" name="$2" h="$3"
    mkdir -p "$prof/.fingerprint/$name-$h" "$prof/deps" "$prof/build/$name-$h"
    printf 'x' > "$prof/.fingerprint/$name-$h/lib-$name"
    printf 'x' > "$prof/deps/lib$name-$h.rlib"
    printf 'x' > "$prof/deps/$name-$h.d"
    printf 'x' > "$prof/deps/$name-$h.cgu0.rcgu.o"
  }
  read_unit() { cat "$1/.fingerprint/$2-$3/lib-$2" > /dev/null; }

  # A filesystem where reads leave access times alone would make `trim` a
  # no-op in the job too, so the self-test fails rather than skips.
  if ! atime_probe "$base"; then
    say "self-test FAILED: access times do not update on $base's filesystem"
    rm -rf "$base"
    return 1
  fi

  # Case 1: live and stale generations; the stale one goes, the live one stays.
  mkunit "$t/debug" foo 0123456789abcdef
  mkunit "$t/debug" foo fedcba9876543210
  mkunit "$t/debug" bar 00000000000000aa
  printf 'x' > "$t/debug/libfoo.rlib"            # uplifted copy: no hash
  printf 'x' > "$t/debug/deps/orphan-1111111111111111"  # no fingerprint at all
  mkdir -p "$t/debug/incremental/foo-abc"
  mkunit "$t/tmp/cli-scaffold-target/debug" node 2222222222222222
  mkunit "$t/tmp/gone-scaffold-target/debug" node 3333333333333333
  printf 'x' > "$t/tmp/scratch.txt"
  cmd_age "$t" > /dev/null
  read_unit "$t/debug" foo 0123456789abcdef
  read_unit "$t/debug" bar 00000000000000aa
  read_unit "$t/tmp/cli-scaffold-target/debug" node 2222222222222222
  cmd_trim "$t" > /dev/null
  [ -e "$t/debug/deps/libfoo-0123456789abcdef.rlib" ] || fail "case 1: live rlib removed"
  [ -e "$t/debug/deps/foo-0123456789abcdef.cgu0.rcgu.o" ] || fail "case 1: live object removed"
  [ -d "$t/debug/.fingerprint/foo-0123456789abcdef" ] || fail "case 1: live fingerprint removed"
  [ -d "$t/debug/build/bar-00000000000000aa" ] || fail "case 1: live build dir removed"
  [ ! -e "$t/debug/deps/libfoo-fedcba9876543210.rlib" ] || fail "case 1: stale rlib kept"
  [ ! -e "$t/debug/deps/foo-fedcba9876543210.d" ] || fail "case 1: stale dep-info kept"
  [ ! -d "$t/debug/.fingerprint/foo-fedcba9876543210" ] || fail "case 1: stale fingerprint kept"
  [ ! -d "$t/debug/build/foo-fedcba9876543210" ] || fail "case 1: stale build dir kept"
  [ ! -e "$t/debug/deps/orphan-1111111111111111" ] || fail "case 1: orphan artifact kept"
  [ -e "$t/debug/libfoo.rlib" ] || fail "case 1: unhashed uplifted file removed"
  [ ! -d "$t/debug/incremental" ] || fail "case 1: incremental kept"
  [ -e "$t/tmp/cli-scaffold-target/debug/deps/libnode-2222222222222222.rlib" ] || fail "case 1: live scaffold unit removed"
  [ ! -e "$t/tmp/gone-scaffold-target" ] || fail "case 1: unread scaffold target kept"
  [ ! -e "$t/tmp/scratch.txt" ] || fail "case 1: tmp scratch file kept"

  # Case 2: nothing read since `age` (the guard): nothing is deleted.
  rm -rf "$t"; mkunit "$t/debug" foo 0123456789abcdef
  cmd_age "$t" > /dev/null
  cmd_trim "$t" > /dev/null
  [ -e "$t/debug/deps/libfoo-0123456789abcdef.rlib" ] || fail "case 2: no-live guard deleted"

  # Case 3: live fingerprints whose hash no deps/ entry carries: profile skipped.
  rm -rf "$t"; mkunit "$t/debug" foo 0123456789abcdef
  mkunit "$t/debug" old 4444444444444444
  rm -f "$t/debug/deps/libfoo-0123456789abcdef.rlib" "$t/debug/deps/foo-0123456789abcdef.d" \
        "$t/debug/deps/foo-0123456789abcdef.cgu0.rcgu.o"
  cmd_age "$t" > /dev/null
  read_unit "$t/debug" foo 0123456789abcdef
  cmd_trim "$t" > /dev/null
  [ -e "$t/debug/deps/libold-4444444444444444.rlib" ] || fail "case 3: naming guard deleted"

  # Case 4: `age` never ran (fresh access times everywhere): nothing is deleted.
  rm -rf "$t"; mkunit "$t/debug" foo 0123456789abcdef; mkunit "$t/debug" old 4444444444444444
  cat "$t/debug/.fingerprint/old-4444444444444444/lib-old" > /dev/null
  cmd_trim "$t" > /dev/null
  [ -e "$t/debug/deps/libold-4444444444444444.rlib" ] || fail "case 4: unaged tree trimmed"

  rm -rf "$base"
  if [ "$fails" -ne 0 ]; then
    return 1
  fi
  say "self-test: OK (4 cases)"
  return 0
}

case "${1:-}" in
  age)  [ $# -eq 2 ] || { echo "usage: $0 age <target-dir>" >&2; exit 2; }; cmd_age "$2" ;;
  trim) [ $# -eq 2 ] || { echo "usage: $0 trim <target-dir>" >&2; exit 2; }; cmd_trim "$2" ;;
  --self-test) self_test; exit $? ;;
  *) echo "usage: $0 age|trim <target-dir> | --self-test" >&2; exit 2 ;;
esac
exit 0
