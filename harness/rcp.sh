#!/usr/bin/env bash
# SPDX-License-Identifier: AGPL-3.0-only
# The program `cerulion rcp --sim` and `cerulion rcp bench` run. cerulion finds this folder and starts this script
# under bash with the terminal attached:
#
#   rcp.sh [--home DIR] sim [--http ADDR] [--task NAME]
#   rcp.sh [--home DIR] bench --model MODEL [--base-url URL] [--key-env NAME] [--cell ID] [--out DIR]
#   rcp.sh [--home DIR] verify
#
# sim     serves the harness's MCP tools with a simulated robot (playground/): over stdio, or over streamable HTTP
#         at ADDR. --task NAME picks the task the robot is set up for (the server's --state).
# bench   runs one benchmark cell with and without the harness: python -m cerulion_harness run-cell.
# verify  checks the measured code against its manifests (cerulion_harness.verify), exit 0 only when every file
#         matches. It installs nothing and needs only python3.
#
# sim and bench run install.sh first when install.sh --check finds a step missing, so the simulator and the
# benchmark scenes are installed once. The install's progress goes to stderr: stdout stays the MCP stream.
#
# --home DIR  where this folder is copied, once, when this user cannot write to it (a system package), because the
#             install writes inside the folder it runs from. cerulion passes harness/<version> under its own config
#             directory, so each release keeps its own copy.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd -P)
USAGE="usage: rcp.sh [--home DIR] sim [--http ADDR] [--task NAME] | bench --model MODEL [...] | verify"
usage() { echo "$USAGE" >&2; exit 2; }

COPY_HOME=
if [ "${1:-}" = --home ]; then
  [ $# -ge 2 ] || usage
  COPY_HOME=$2
  shift 2
fi
[ $# -ge 1 ] || usage
VERB=$1
shift

# The folder sim and bench run from: this one, or its copy when this user cannot write here.
RUN_FROM=$HERE
if [ ! -w "$HERE" ] && [ -n "$COPY_HOME" ]; then
  RUN_FROM=$COPY_HOME
fi

# sim's flags, as the server reads them (--task is the server's --state).
TRANSPORT=--stdio
ADDR=
TASK=
read_sim_flags() {
  while [ $# -gt 0 ]; do
    case "$1" in
      --http) [ $# -ge 2 ] || usage; TRANSPORT=--http; ADDR=$2; shift 2 ;;
      --task) [ $# -ge 2 ] || usage; TASK=$2; shift 2 ;;
      *) usage ;;
    esac
  done
}

# The two refusals run-cell gives before it starts, given here before a first install, which takes minutes.
check_bench_endpoint() {
  local base_url=${OPENAI_BASE_URL:-} key_env=OPENAI_API_KEY
  while [ $# -gt 0 ]; do
    case "$1" in
      --base-url) [ $# -ge 2 ] || usage; base_url=$2; shift 2 ;;
      --key-env) [ $# -ge 2 ] || usage; key_env=$2; shift 2 ;;
      *) shift ;;
    esac
  done
  [ -n "$base_url" ] || { echo "no endpoint: pass --base-url or set OPENAI_BASE_URL" >&2; exit 2; }
  case "$key_env" in
    ''|[0-9]*|*[!A-Za-z0-9_]*) echo "--key-env: '$key_env' is not an environment variable name" >&2; exit 2 ;;
  esac
  [ -n "${!key_env:-}" ] || {
    echo "no key: set the environment variable $key_env (or name another with --key-env)" >&2
    exit 2
  }
}

case "$VERB" in
  verify)
    [ $# -eq 0 ] || usage
    # The folder that runs: the copy once there is one, else this folder, which the copy is made from byte for byte.
    [ -f "$RUN_FROM/rcp.sh" ] || RUN_FROM=$HERE
    PY=$RUN_FROM/venv/bin/python
    if [ ! -x "$PY" ]; then
      PY=$(command -v python3 || true)
      [ -n "$PY" ] || { echo "verify needs python3 on PATH, or a finished install" >&2; exit 2; }
    fi
    # cerulion_harness.verify.verify(folder) returns (problems, facts); every problem is printed and fails the run.
    exec "$PY" -I -B -c '
import os, sys
sys.path.insert(0, os.path.join(sys.argv[1], "src"))
from cerulion_harness import verify
problems, facts = verify.verify(sys.argv[1])
for arm in sorted(facts):
    fact = facts[arm]
    print(arm + ": snapshot " + str(fact.get("snapshot")) + ", " + str(fact.get("shipped_files")) + " shipped files")
for problem in problems:
    print("PROBLEM: " + problem, file=sys.stderr)
if problems:
    print(f"verify: {len(problems)} problem(s); this folder is not the measured code")
    sys.exit(1)
print("verify: ok, this folder holds the code the benchmark measured")
' "$RUN_FROM"
    ;;
  sim) read_sim_flags "$@" ;;
  bench) check_bench_endpoint "$@" ;;
  *) usage ;;
esac

if [ "$RUN_FROM" != "$HERE" ]; then
  if [ ! -f "$RUN_FROM/rcp.sh" ]; then
    echo "$HERE is read-only for this user: copying it once to $RUN_FROM, where the install goes" >&2
    mkdir -p "$(dirname "$RUN_FROM")"
    TMP=$RUN_FROM.partial.$$
    rm -rf "$TMP"
    cp -R "$HERE" "$TMP"
    chmod -R u+w "$TMP"
    if [ -e "$RUN_FROM" ]; then rm -rf "$TMP"; else mv "$TMP" "$RUN_FROM"; fi
  fi
  exec bash "$RUN_FROM/rcp.sh" "$VERB" "$@"
fi
[ -w "$HERE" ] || {
  echo "this user cannot write to $HERE, where the install goes; run it through cerulion rcp, which names a" \
    "folder to copy it to, or point CERULION_HARNESS_DIR at a copy you own" >&2
  exit 2
}

if ! bash "$HERE/install.sh" --check >/dev/null 2>&1; then
  echo "First run: installing the simulator and the benchmark scenes into $HERE, once." >&2
  bash "$HERE/install.sh" >&2 || {
    code=$?
    echo "the install stopped (exit $code); run the same command again to resume it" >&2
    exit "$code"
  }
fi
PY=$HERE/venv/bin/python

if [ "$VERB" = bench ]; then
  exec "$PY" -m cerulion_harness run-cell "$@"
fi
set -- "$TRANSPORT"
[ -z "$ADDR" ] || set -- "$@" "$ADDR"
[ -z "$TASK" ] || set -- "$@" --state "$TASK"
exec "$PY" "$HERE/playground" "$@"
