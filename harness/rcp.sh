#!/usr/bin/env bash
# SPDX-License-Identifier: AGPL-3.0-only
# The program `cerulion rcp --sim` and `cerulion rcp bench` run. cerulion finds this folder and starts this script
# under bash with the terminal attached:
#
#   rcp.sh [--home DIR [--packaged]] sim [--http ADDR] [--task NAME]
#   rcp.sh [--home DIR [--packaged]] bench --model MODEL [--base-url URL] [--key-env NAME] [--cell ID] [--out DIR]
#   rcp.sh [--home DIR [--packaged]] verify
#
# sim     serves the harness's MCP tools with a simulated robot (playground/): over stdio, or over streamable HTTP
#         at ADDR. --task NAME picks the task the robot is set up for (the server's --state).
# bench   runs one benchmark cell with and without the harness: python -m cerulion_harness run-cell.
# verify  checks the measured code against its manifests (cerulion_harness.verify), exit 0 only when every file
#         matches. It installs nothing and needs only python3.
#
# sim and bench run install.sh first when install.sh --check finds a step missing, so the simulator and the
# benchmark scenes are installed once. The install's progress goes to stderr: stdout stays the MCP stream. The
# install runs in this folder with no stdin, as its own process group, and a stop that reaches this script
# (SIGINT, SIGTERM or SIGHUP) is passed on to all of it.
#
# --home DIR  where this folder is copied, once, byte for byte, to run from there, because the install writes inside
#             the folder it runs from: always after --packaged, else only when this user cannot write to this folder.
#             cerulion passes harness/<version> under its own config directory, so each release keeps its own copy.
# --packaged  this folder is the one a release package installs beside cerulion, which nothing writes into, so sim
#             and bench run from the copy even when this user can write here. cerulion passes it for that folder only.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd -P)
USAGE="usage: rcp.sh [--home DIR [--packaged]] sim [--http ADDR] [--task NAME] | bench --model MODEL [...] | verify"
usage() { echo "$USAGE" >&2; exit 2; }

COPY_HOME=
PACKAGED=
if [ "${1:-}" = --home ]; then
  [ $# -ge 2 ] || usage
  COPY_HOME=$2
  shift 2
  if [ "${1:-}" = --packaged ]; then
    [ -n "$COPY_HOME" ] || usage
    PACKAGED=1
    shift
  fi
fi
[ $# -ge 1 ] || usage
VERB=$1
shift

# The folder sim and bench run from: this one, or its copy when this folder is a release package's or this user
# cannot write here. A copy that exists already is named by its resolved path, so a --home that names this very
# folder compares equal to it below.
RUN_FROM=$HERE
if [ -n "$COPY_HOME" ] && { [ -n "$PACKAGED" ] || [ ! -w "$HERE" ]; }; then
  RUN_FROM=$COPY_HOME
  [ ! -d "$RUN_FROM" ] || RUN_FROM=$(cd "$RUN_FROM" && pwd -P)
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

# A stop that reaches this script alone, as cerulion forwards one, would otherwise be lost: bash acts on a signal
# only once the command it waits on returns, and that command never got the signal. So the install and its check run
# as their own process group (run_stoppable), and the trap sends the signal to that whole group; once the install has
# ended the script exits with 128 + the signal's number. A stop at any other point ends the script as soon as the
# command running then returns, and a copy it cut short is removed.
CHILD=
STARTING=
STOPPED=
STOP_SIGNAL=
PARTIAL=
pass_stop() {  # to CHILD's whole group
  kill -s "$STOP_SIGNAL" -- "-$CHILD" 2>/dev/null || true
  kill -s CONT -- "-$CHILD" 2>/dev/null || true  # a member stopped by the terminal acts on it too
}
on_stop() {  # SIGNAL CODE
  STOP_SIGNAL=$1
  STOPPED=$2
  if [ -n "$CHILD" ]; then
    pass_stop
    return 0
  fi
  # While run_stoppable starts its command, the stop is passed on as soon as the command's group exists.
  [ -n "$STARTING" ] || exit "$2"
}
trap 'on_stop INT 130' INT
trap 'on_stop TERM 143' TERM
trap 'on_stop HUP 129' HUP
trap '[ -z "$PARTIAL" ] || rm -rf "$PARTIAL"' EXIT

# run_stoppable CMD...: run CMD in this folder, so its Python imports nothing from the directory cerulion was
# started in, as its own process group with no stdin (over stdio, stdin is the MCP stream the server reads). With
# no terminal to answer on, git fails where it would ask for a login (GIT_TERMINAL_PROMPT=0) instead of waiting.
# Returns CMD's code, or the stop's code once a stopped CMD has ended.
run_stoppable() {
  STARTING=1
  set -m
  (cd "$HERE" && export GIT_TERMINAL_PROMPT=0 && exec "$@") </dev/null &
  CHILD=$!
  set +m
  STARTING=
  [ -z "$STOPPED" ] || pass_stop
  local code=0
  wait "$CHILD" || code=$?
  while [ -n "$STOPPED" ] && kill -0 "$CHILD" 2>/dev/null; do
    code=0
    wait "$CHILD" || code=$?
  done
  CHILD=
  return "${STOPPED:-$code}"
}

if [ "$RUN_FROM" != "$HERE" ]; then
  if [ ! -f "$RUN_FROM/rcp.sh" ]; then
    if [ -n "$PACKAGED" ]; then
      echo "$HERE is a release package's folder, which nothing writes into: copying it once to $RUN_FROM, where" \
        "the install goes" >&2
    else
      echo "$HERE is read-only for this user: copying it once to $RUN_FROM, where the install goes" >&2
    fi
    mkdir -p "$(dirname "$RUN_FROM")"
    PARTIAL=$RUN_FROM.partial.$$
    rm -rf "$PARTIAL"
    cp -R "$HERE" "$PARTIAL"
    chmod -R u+w "$PARTIAL"
    if [ -e "$RUN_FROM" ]; then rm -rf "$PARTIAL"; else mv "$PARTIAL" "$RUN_FROM"; fi
    PARTIAL=
  fi
  exec bash "$RUN_FROM/rcp.sh" "$VERB" "$@"
fi
[ -z "$PACKAGED" ] || {
  echo "--home names $HERE itself, a release package's folder, which nothing writes into; name another folder" >&2
  exit 2
}
[ -w "$HERE" ] || {
  echo "this user cannot write to $HERE, where the install goes; run it through cerulion rcp, which names a" \
    "folder to copy it to, or point CERULION_HARNESS_DIR at a copy you own" >&2
  exit 2
}

if ! run_stoppable bash "$HERE/install.sh" --check >/dev/null 2>&1; then
  [ -z "$STOPPED" ] || exit "$STOPPED"
  echo "First run: installing the simulator and the benchmark scenes into $HERE, once." >&2
  run_stoppable bash "$HERE/install.sh" >&2 || {
    code=$?
    echo "the install stopped (exit $code); run the same command again to resume it" >&2
    exit "$code"
  }
fi
PY=$HERE/venv/bin/python

if [ "$VERB" = bench ]; then
  # -P: the directory cerulion was started in, where the results go, is not on the module path, so nothing in it
  # is imported in place of the harness's own modules.
  exec "$PY" -P -m cerulion_harness run-cell "$@"
fi
set -- "$TRANSPORT"
[ -z "$ADDR" ] || set -- "$@" "$ADDR"
[ -z "$TASK" ] || set -- "$@" --state "$TASK"
exec "$PY" "$HERE/playground" "$@"
