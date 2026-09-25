#!/usr/bin/env bash
# Full wheel path: build the wheel, install into a FRESH venv, run the
# fixture round-trip from /tmp, diff against the literal expected output.
#
#   E2E_WHEEL=/path/to/wheel.whl   skip the build, test that wheel instead
set -euo pipefail

cd "$(dirname "$0")/../../.."
REPO="$PWD"
# E2E_WHEEL is resolved against the caller's cwd (repo root here) BEFORE
# we descend into crates/cerulion_py/.
[ -n "${E2E_WHEEL:-}" ] && E2E_WHEEL="$(cd "$(dirname "$E2E_WHEEL")" && pwd)/$(basename "$E2E_WHEEL")"
cd crates/cerulion_py
export RUSTUP_TOOLCHAIN=1.93.0

# Temp roots are cleaned on ANY exit path (error under `set -e` and
# INT/TERM via the same trap); a caller-provided E2E_WHEEL and the
# repo's dist/ are never deleted.
BUILD_TMP="$(mktemp -d)"
TEST_TMP="$(mktemp -d)"
trap 'rm -rf "$BUILD_TMP" "$TEST_TMP"' EXIT
trap 'exit 130' INT TERM

if [ -z "${E2E_WHEEL:-}" ]; then
    BUILD_VENV="$BUILD_TMP/venv"
    python3 -m venv "$BUILD_VENV"
    "$BUILD_VENV/bin/pip" install -q 'maturin>=1.15,<2'
    mkdir -p dist
    "$BUILD_VENV/bin/maturin" build --release --locked -m Cargo.toml -o dist
    WHEEL="$(ls -t dist/cerulion-*.whl | head -1)"
else
    WHEEL="$E2E_WHEEL"
fi
[ -f "$WHEEL" ] || { echo "wheel not found: $WHEEL" >&2; exit 1; }

cargo build --release --locked -p cerulion_py_fixtures
export CERULION_PY_FIXTURE="$REPO/crates/cerulion_py/target/release/cerulion_py_fixture"

TEST_VENV="$TEST_TMP/venv"
python3 -m venv "$TEST_VENV"
"$TEST_VENV/bin/pip" install -q --no-deps "$WHEEL" 'numpy>=1.26'

# Linker-mitigation regression check: cerulion_py/.cargo/config.toml
# hides iceoryx2's `__internal_default_logger` (and every other static-dep
# symbol) from the wheel's dynamic symbol table. A wheel built WITHOUT
# those rustflags - e.g. maturin run from the repo root, above the
# workspace's .cargo dir - re-exports them, and two such .so's in one
# process interpose on each other's iceoryx2.
SO="$(find "$TEST_VENV" -name '_native*.so' -print -quit)"
[ -n "$SO" ] || { echo "installed _native .so not found under $TEST_VENV" >&2; exit 1; }
# Linux only: ELF resolves symbols in one flat namespace. Mach-O binds each
# symbol to the image that defines it (two-level namespace), so two
# extensions on macOS never interpose and the flags above are Linux-only.
if [ "$(uname -s)" = Linux ]; then
    # `nm` must succeed (a missing/incompatible nm must fail closed, not
    # silently pass the guard); grep's no-match is the legitimate empty case.
    DYN_SYMS="$(nm -D --defined-only "$SO")"
    LEAKED="$(echo "$DYN_SYMS" | grep 'iceoryx2\|__internal_default_logger' || true)"
    if [ -n "$LEAKED" ]; then
        echo "E2E FAIL: wheel exports hidden transport symbols (was it built outside cerulion_py/?):" >&2
        echo "$LEAKED" >&2
        exit 1
    fi
fi

cd /tmp
OUT="$("$TEST_VENV/bin/python" "$REPO/crates/cerulion_py/scripts/e2e_roundtrip.py")"
EXPECTED='frame seq=0 schema_hash=305419896 timestamp_ns=1000 total_size=96 payload_len=64 fnv1a64=dc73a7cb467baca5
frame seq=1 schema_hash=305419896 timestamp_ns=1000 total_size=96 payload_len=64 fnv1a64=d4f6e5f13c429765
frame seq=2 schema_hash=305419896 timestamp_ns=1000 total_size=96 payload_len=64 fnv1a64=9f8afc9eacf610a5
fixture: frame seq=0 schema_hash=305419896 timestamp_ns=77 total_size=96 body_len=64 fnv1a64=dc73a7cb467baca5'
if [ "$OUT" != "$EXPECTED" ]; then
    echo "E2E MISMATCH" >&2
    diff <(echo "$EXPECTED") <(echo "$OUT") >&2
    exit 1
fi
echo "E2E OK"
