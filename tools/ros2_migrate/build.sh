#!/usr/bin/env bash
# Build cerulion-ros2-migrate-clang inside the ros2-bench Jazzy container (the
# repo's ROS/clang toolchain home, see tools/ros2_toolchain/Dockerfile). This is a
# standalone clang LibTooling tool; it is deliberately NOT part of the Rust
# workspace build, so the workspace never grows a libclang dependency.
#
# Usage (inside the container, from this directory):
#   ./build.sh                         # -> ./cerulion-ros2-migrate-clang
#   ./build.sh --mutant DROP_ESCAPE_CHECK
#       # -> ./cerulion-ros2-migrate-clang-mutant-drop_escape_check
#       # (run_matrix.sh uses these; NEVER ship a mutant build)
#   ./build.sh --out-dir DIR [--mutant NAME]
#       # -> DIR/<the same name>. run_matrix.sh builds every mutant into a
# # run-PRIVATE directory: a mutant at
#       # the shared name here would be built, then identity-bound, and a
#       # concurrent run could replace it in that gap — the first run would
#       # bind the second run's artifact as its own and delete it at cleanup.
#       # A pathname no other run can name has no such gap. DIR must already
#       # exist (this script creates nothing but the binary) and is taken
#       # relative to THIS directory when relative, since the build runs here.
set -euo pipefail
cd "$(dirname "$0")"

# STRICT argv: `--mutant NAME` and `--out-dir DIR`, each at most once, in
# either order, and nothing else. An unknown or misspelled flag must fail
# LOUDLY — falling through with MUTANT empty would build the UNMUTATED
# production prover while the caller believes a mutant was built (a
# silently-vacuous mutation run), and falling through with OUT_DIR empty
# would build at the SHARED name while the caller believes the artifact
# is private.
usage() {
    echo "usage: ./build.sh [--out-dir DIR] [--mutant NAME]   (NAME e.g. \
DROP_ESCAPE_CHECK; DIR must already exist)" >&2
    exit 2
}
MUTANT=""
OUT_DIR=""
while [ $# -gt 0 ]; do
    case "$1" in
        --mutant)
            [ -z "$MUTANT" ] || { echo "error: --mutant given twice" >&2; usage; }
            [ $# -ge 2 ] || { echo "error: --mutant needs a name" >&2; usage; }
            MUTANT="$2"
            case "$MUTANT" in
                ''|-*) echo "error: --mutant needs a name (got '${MUTANT}')" >&2; usage ;;
                *[!A-Z0-9_]*) echo "error: mutant name '${MUTANT}' must be [A-Z0-9_]" >&2; usage ;;
            esac
            shift 2
            ;;
        --out-dir)
            [ -z "$OUT_DIR" ] || { echo "error: --out-dir given twice" >&2; usage; }
            [ $# -ge 2 ] || { echo "error: --out-dir needs a directory" >&2; usage; }
            OUT_DIR="$2"
            case "$OUT_DIR" in
                ''|-*) echo "error: --out-dir needs a directory (got '${OUT_DIR}')" >&2; usage ;;
            esac
            # An existing directory, reached without following a symlink: a
            # link here is a redirect of where a mutant lands, and a caller
            # that asked for a private directory did not ask for that.
            if [ -L "$OUT_DIR" ] || [ ! -d "$OUT_DIR" ]; then
                echo "error: --out-dir '${OUT_DIR}' is not an existing directory (it is \
$(if [ -L "$OUT_DIR" ]; then echo a symlink; elif [ -e "$OUT_DIR" ]; then echo "not a directory"; else echo missing; fi)) \
— this script creates nothing but the binary" >&2
                exit 2
            fi
            shift 2
            ;;
        *)
            echo "error: unknown argument '$1' (expected [--out-dir DIR] [--mutant NAME])" >&2
            usage
            ;;
    esac
done

LLVM_CONFIG=""
for cand in llvm-config-18 llvm-config; do
    if command -v "$cand" >/dev/null 2>&1; then
        LLVM_CONFIG="$cand"
        break
    fi
done
if [ -z "$LLVM_CONFIG" ]; then
    echo "error: llvm-config not found. Inside the ros2-bench container run:" >&2
    echo "  apt-get update && apt-get install -y --no-install-recommends llvm-dev clang libclang-dev" >&2
    exit 69
fi

INCLUDEDIR=$("$LLVM_CONFIG" --includedir)
LIBDIR=$("$LLVM_CONFIG" --libdir)
if [ ! -e "$INCLUDEDIR/clang/Tooling/Tooling.h" ]; then
    echo "error: clang LibTooling headers not found under $INCLUDEDIR." >&2
    echo "  apt-get update && apt-get install -y --no-install-recommends libclang-dev" >&2
    exit 69
fi

# Ubuntu links clang tools against the monolithic libclang-cpp; prefer the
# unversioned symlink, fall back to the versioned shared object.
CLANG_CPP_LIB="-lclang-cpp"
if [ ! -e "$LIBDIR/libclang-cpp.so" ]; then
    versioned=$(ls "$LIBDIR"/libclang-cpp.so.* 2>/dev/null | head -1 || true)
    if [ -n "$versioned" ]; then
        CLANG_CPP_LIB="$versioned"
    else
        echo "error: libclang-cpp not found under $LIBDIR." >&2
        echo "  apt-get update && apt-get install -y --no-install-recommends libclang-dev" >&2
        exit 69
    fi
fi

OUT="cerulion-ros2-migrate-clang"
DEFINE=()
if [ -n "$MUTANT" ]; then
    lower=$(printf '%s' "$MUTANT" | tr '[:upper:]' '[:lower:]')
    OUT="$OUT-mutant-$lower"
    DEFINE=("-DCERULION_MIGRATE_MUTANT_${MUTANT}")
fi
# The name is the same wherever it lands: the registry's stale-list glob and
# the mutant's own `-mutant-<name>` suffix are what identify it, in the
# shared directory or a private one.
OUT_PATH="$OUT"
if [ -n "$OUT_DIR" ]; then
    OUT_PATH="$OUT_DIR/$OUT"
fi

# The prover's total-switch guard must be ARMED, and that is not a matter
# of tidiness: the ScanResult dispatch is a total switch with no `default`,
# so adding a verdict without a branch must FAIL THE BUILD. `-Wall` alone
# only warns, and the branch it would fall through to is the rewrite path.
# Both other switches in the tool carry a `default:`, so this cannot reach
# them.
#
# Two things protect it, because ORDER ALONE IS NOT ENOUGH (a
# review MEASURED this on clang 17 against a switch missing an enumerator):
#
#   -Wno-switch -Werror=switch   -> errors   (armed: last -W wins)
#   -Werror=switch -Wno-switch   -> silent   (the bug the order fixes)
#   -w -Werror=switch            -> SILENT   (order does NOT save it)
#   -Werror=switch -w            -> silent
#
# So `-w` is not an ordinary `-W` flag — it suppresses in EITHER position.
# Hence:
#
#  (1) every llvm-config expansion is captured and REFUSED if it carries one
#      of the warning-suppressing spellings listed below. It is a BLOCKLIST,
#      not a proof: a future spelling would pass. What it does cover is `-w`,
#      which no ordering can survive, and the expansions that land AFTER the
#      promotion on the command line (--ldflags/--libs/--system-libs), which
#      no ordering of this one token could protect either. `@file` response
#      files are refused too, since the driver expands them after this gate
#      has looked.
#  (2) `-Werror=switch` still sits AFTER `$LLVM_CXXFLAGS`, so the ordinary
#      `-Wno-switch` case is armed even if the refusal in (1) is ever
#      narrowed.
#
# `-Wall` deliberately stays BEFORE the expansion: it is a breadth flag,
# not a gate, and nothing downstream depends on any single warning it
# enables. Ubuntu 24.04's llvm-config-18 (the ros2-bench container's) emitted
# no -W flags when this was written; nothing pins that, which is exactly
# why the refusal above carries the guarantee rather than the observation.
LLVM_CXXFLAGS=$("$LLVM_CONFIG" --cxxflags)
LLVM_LDFLAGS=$("$LLVM_CONFIG" --ldflags)
LLVM_LIBS=$("$LLVM_CONFIG" --libs)
LLVM_SYSLIBS=$("$LLVM_CONFIG" --system-libs)
for flag in $LLVM_CXXFLAGS $LLVM_LDFLAGS $LLVM_LIBS $LLVM_SYSLIBS; do
    case "$flag" in
        -w|-Wno-switch|-Wno-switch=*|-Wno-everything|-Wno-error|-Wno-error=*|@*)
            echo "error: '$LLVM_CONFIG' emits '$flag', which would make \
-Werror=switch INERT — the prover's total-switch guard would then look armed \
while a missing ScanResult branch fell through to the rewrite path. Refusing \
to build a prover whose refusal gate is not armed." >&2
            exit 2
            ;;
    esac
done

# shellcheck disable=SC2086  # the llvm-config flag strings are split on purpose
clang++ -O2 -std=c++17 -Wall $LLVM_CXXFLAGS -Werror=switch \
    ${DEFINE[@]+"${DEFINE[@]}"} \
    cerulion_ros2_migrate_clang.cpp \
    -o "$OUT_PATH" \
    $LLVM_LDFLAGS "$CLANG_CPP_LIB" $LLVM_LIBS \
    $LLVM_SYSLIBS

if [ -n "$OUT_DIR" ]; then
    echo "built: $OUT_PATH"
else
    echo "built: $(pwd)/$OUT"
fi
