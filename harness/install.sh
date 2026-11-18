#!/usr/bin/env bash
# SPDX-License-Identifier: AGPL-3.0-only
# The harness's one-time install: the simulator, the benchmark scenes and the episode server, built inside this
# folder. cerulion rcp --sim and cerulion rcp bench run it before they start: a step that is installed and
# current is skipped, so a run after a finished install returns in under a second.
#
#   install.sh [--bench-from DIR] [--check]
#
# It builds, inside this folder:
#   bx/libero-env              Python 3.10: the simulator (requirements/sim-py310.txt and CPU torch)
#   bx/LIBERO, bx/LIBERO-PRO   the benchmark at its pinned revisions (benchmarks/fetch.sh)
#   venv                       Python 3.12: the episode server (requirements/server-py312.txt), with this
#                              folder's src/ on its path, so venv/bin/python -m cerulion_harness runs
# and checks that the robot of the default start state has a body record on this machine (measured once where
# none is stored). Nothing it writes is tracked by git: the scenes are fetched, never committed.
#
# --bench-from DIR  DIR already holds LIBERO and LIBERO-PRO as benchmarks/fetch.sh leaves them. They are linked,
#                   not fetched again; the 160 pinned data files are still checked.
# --check           write nothing: exit 0 when every step is installed and current, 1 when one is not.
#
# Needs git, curl and about 4 GB of disk (5 GB free to start). uv (https://docs.astral.sh/uv/) is used from PATH;
# without one, uv 0.13.7 is fetched into bx/uv and checked against its pinned SHA-256. No GPU. Linux also needs
# the CPU renderer (Ubuntu: sudo apt-get install libosmesa6 libegl1 libgl1 libglib2.0-0).
set -euo pipefail
# pwd -P: the folder's real path. The simulator host refuses a benchmark tree it loaded from another path than
# the one it was given, and the benchmark is loaded by its real path, so a folder reached through a link (/tmp
# on macOS, a linked home folder) is named by its real path.
HARNESS=$(cd "$(dirname "$0")" && pwd -P)
USAGE="usage: install.sh [--bench-from DIR] [--check]"
FROM=
CHECK=
while [ $# -gt 0 ]; do
  case "$1" in
    --bench-from)
      [ $# -ge 2 ] || { echo "$USAGE" >&2; exit 2; }
      [ -d "$2" ] || { echo "--bench-from: $2 is not a folder" >&2; exit 2; }
      FROM=$(cd "$2" && pwd -P); shift 2 ;;
    --check) CHECK=1; shift ;;
    -h|--help) sed -n '3,23p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "$USAGE" >&2; exit 2 ;;
  esac
done
OS=$(uname -s); ARCH=$(uname -m)
case "$OS/$ARCH" in
  Darwin/arm64) UV_TARGET=aarch64-apple-darwin
    UV_SHA=81b811b57e4624df7ed3aac148e0359100d9962f3d9dce00aec533dc2e086731 ;;
  Linux/x86_64) UV_TARGET=x86_64-unknown-linux-gnu
    UV_SHA=c5221ff41b7892bfc223543883600125f6a47fe845093e0f5bdff4f359196d83 ;;
  Linux/aarch64|Linux/arm64) UV_TARGET=aarch64-unknown-linux-gnu
    UV_SHA=71279f8d3c67241e807b9d8db872d1bf541579fb9f08b7d1eb1682196d1e57dc ;;
  *) echo "not supported: $OS on $ARCH (supported: macOS on Apple silicon, Linux on x86-64 or arm64)" >&2; exit 2 ;;
esac
UV_VERSION=0.13.7
BX=$HARNESS/bx
SIM_PY=$BX/libero-env/bin/python
VENV_PY=$HARNESS/venv/bin/python

sha256_stdin() {  # the SHA-256 of standard input, as hex
  if command -v sha256sum >/dev/null; then sha256sum | cut -d' ' -f1; else shasum -a 256 | cut -d' ' -f1; fi
}
sha256_of() {  # FILE...: one SHA-256 over the files' bytes in order
  cat "$@" | sha256_stdin
}
# Each step's stamp holds the SHA-256 of what the step was built from; a stamp that matches skips the step.
SIM_IN=$(sha256_of "$HARNESS/requirements/sim-py310.txt" "$HARNESS/requirements/sim-torch-cpu.txt")
SCENES_IN=$(sha256_of "$HARNESS/benchmarks/fetch.sh" "$HARNESS/benchmarks/LIBERO-PRO-data.files.txt" \
  "$HARNESS/benchmarks/LIBERO-PRO-data.sha256")
SERVER_IN=$(printf '%s\n' "$(sha256_of "$HARNESS/requirements/server-py312.txt")" "$HARNESS" "$OS" | sha256_stdin)
current() {  # NAME DIGEST: the step's stamp matches
  [ -f "$BX/.step-$1" ] && [ "$(cat "$BX/.step-$1")" = "$2" ]
}

if [ -n "$CHECK" ]; then
  missing=0
  if ! { current sim "$SIM_IN" && [ -x "$SIM_PY" ]; }; then
    echo "not installed: the simulator environment ($BX/libero-env)"; missing=1
  fi
  if ! { current scenes "$SCENES_IN" && [ -d "$BX/LIBERO-PRO/libero/libero" ]; }; then
    echo "not installed: the benchmark scenes ($BX)"; missing=1
  fi
  if ! { current server "$SERVER_IN" && [ -x "$VENV_PY" ]; }; then
    echo "not installed: the episode server environment ($HARNESS/venv)"; missing=1
  fi
  if [ "$missing" = 0 ]; then echo "installed: $HARNESS"; fi
  exit "$missing"
fi

[ -w "$HARNESS" ] || {
  echo "the install writes bx/, venv/ and this machine's body records inside $HARNESS, which this user cannot write" >&2
  exit 2
}
for tool in git curl; do
  command -v "$tool" >/dev/null || { echo "missing: $tool (install it, then run this again)" >&2; exit 2; }
done
if [ "$OS" = Linux ] && command -v ldconfig >/dev/null; then
  # The whole list is read before it is searched: grep -q on a pipe can end the writer early under pipefail.
  # An empty list (no library cache) judges nothing; the simulator start of step 4 still would.
  LIBS=$(ldconfig -p 2>/dev/null || true)
  case "$LIBS" in
    ''|*libOSMesa.so*) ;;
    *) echo "missing: the CPU renderer libOSMesa, which the simulator renders with (Ubuntu:" \
         "sudo apt-get install libosmesa6 libegl1 libgl1 libglib2.0-0)" >&2
       exit 2 ;;
  esac
fi
NEED_KB=$((5 * 1024 * 1024))
if ! { current sim "$SIM_IN" && current scenes "$SCENES_IN" && current server "$SERVER_IN"; }; then
  FREE_KB=$(df -Pk "$HARNESS" | awk 'NR == 2 { print $4 }')
  if [ -n "$FREE_KB" ] && [ "$FREE_KB" -lt "$NEED_KB" ]; then
    echo "the install needs about 4 GB and refuses to start with less than 5 GB free; $HARNESS has" \
      "$((FREE_KB / 1024)) MB free" >&2
    exit 2
  fi
fi
mkdir -p "$BX"
T0=$(date +%s)
took() { echo "      done in $(( $(date +%s) - $1 )) s"; }

UV=$(command -v uv || true)
if [ -z "$UV" ]; then
  UV=$BX/uv/uv
  if [ ! -x "$UV" ]; then
    echo "uv is not on PATH: fetching uv $UV_VERSION into $BX/uv"
    mkdir -p "$BX/uv"
    curl -fsSL --retry 5 --retry-delay 5 -o "$BX/uv/uv.tar.gz" \
      "https://github.com/astral-sh/uv/releases/download/$UV_VERSION/uv-$UV_TARGET.tar.gz"
    [ "$(sha256_of "$BX/uv/uv.tar.gz")" = "$UV_SHA" ] || {
      echo "the uv download does not match its pinned SHA-256; nothing was installed from it" >&2
      rm -f "$BX/uv/uv.tar.gz"; exit 1
    }
    tar -xzf "$BX/uv/uv.tar.gz" -C "$BX/uv" --strip-components 1
    rm -f "$BX/uv/uv.tar.gz"
  fi
fi

echo "[1/4] the simulator environment (Python 3.10, about 2 GB)"
if current sim "$SIM_IN" && [ -x "$SIM_PY" ]; then
  echo "      installed"
else
  t=$(date +%s)
  rm -f "$BX/.step-sim"
  [ -x "$SIM_PY" ] || "$UV" venv --python 3.10 "$BX/libero-env"
  "$UV" pip install --python "$SIM_PY" -r "$HARNESS/requirements/sim-py310.txt"
  if [ "$OS" = Darwin ]; then
    # macOS wheels of torch carry no "+cpu" tag and live on the default index: the same version, CPU only.
    "$UV" pip install --python "$SIM_PY" --no-deps \
      "$(sed -n 's/^\(torch==[0-9.]*\)+cpu$/\1/p' "$HARNESS/requirements/sim-torch-cpu.txt")"
  else
    "$UV" pip install --python "$SIM_PY" --no-deps --index-url https://download.pytorch.org/whl/cpu \
      -r "$HARNESS/requirements/sim-torch-cpu.txt"
  fi
  echo "$SIM_IN" > "$BX/.step-sim"
  took "$t"
fi

echo "[2/4] the benchmark scenes at their pinned revisions (about 1.6 GB)"
if current scenes "$SCENES_IN" && [ -d "$BX/LIBERO-PRO/libero/libero" ]; then
  echo "      installed"
else
  t=$(date +%s)
  rm -f "$BX/.step-scenes"
  if [ -n "$FROM" ]; then
    for tree in LIBERO LIBERO-PRO; do
      [ -d "$FROM/$tree/libero/libero" ] || { echo "$FROM/$tree is not a benchmark tree" >&2; exit 2; }
      [ -e "$BX/$tree" ] || ln -s "$FROM/$tree" "$BX/$tree"
    done
    FETCH_LINKED=1 UV="$UV" bash "$HARNESS/benchmarks/fetch.sh" "$BX" "$SIM_PY"
  else
    UV="$UV" bash "$HARNESS/benchmarks/fetch.sh" "$BX" "$SIM_PY"
  fi
  echo "$SCENES_IN" > "$BX/.step-scenes"
  took "$t"
fi

echo "[3/4] the episode server environment (Python 3.12, about 0.5 GB)"
if current server "$SERVER_IN" && [ -x "$VENV_PY" ]; then
  echo "      installed"
else
  t=$(date +%s)
  rm -f "$BX/.step-server"
  [ -x "$VENV_PY" ] || "$UV" venv --python 3.12 "$HARNESS/venv"
  "$UV" pip install --python "$VENV_PY" -r "$HARNESS/requirements/server-py312.txt"
  SITE=$("$VENV_PY" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')
  # This folder's src/ on the environment's path: python -m cerulion_harness runs this folder's code.
  echo "$HARNESS/src" > "$SITE/cerulion_harness.pth"
  if [ "$OS" = Darwin ]; then
    # MuJoCo on macOS refuses the CPU renderer's name: macos/cerulion_harness_macos_gl.py says what these do.
    cp "$HARNESS/macos/cerulion_harness_macos_gl.py" "$SITE/cerulion_harness_macos_gl.py"
    echo "import cerulion_harness_macos_gl" > "$SITE/cerulion_harness_macos_gl.pth"
  fi
  echo "$SERVER_IN" > "$BX/.step-server"
  took "$t"
fi

echo "[4/4] this machine's body record for the default start state"
t=$(date +%s)
"$VENV_PY" -m cerulion_harness.body_record > "$BX/body_record.json" || {
  echo "the body record check failed; its report: $BX/body_record.json" >&2
  exit 1
}
took "$t"
echo "installed in $(( $(date +%s) - T0 )) s: $HARNESS"
