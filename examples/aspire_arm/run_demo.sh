#!/usr/bin/env bash
# Record an ASPIRE episode on the simulated arm, then replay it with no agent.
#
#   1. builds arm_sim and starts `cerulion graph run aspire_arm --record`;
#   2. runs one ASPIRE skill program through the `cerulion` embodiment, which
#      drives the arm only through `cerulion-rcpd` (MCP over stdio);
#   3. stops the graph, so the recorder finalizes the bag;
#   4. checks the receipt log's hash chain;
#   5. replays the bag with `cerulion bag play --resim all --verify`: arm_sim
#      recomputes every answer from the recorded tool calls, with neither ASPIRE
#      nor cerulion-rcpd nor any model running.
#
# Needs `cerulion` and `cerulion-rcpd` on PATH, and an ASPIRE checkout that has
# the `cerulion` embodiment installed into ASPIRE_PYTHON's environment.
#
#   ASPIRE_DIR=~/src/ASPIRE ASPIRE_PYTHON=~/src/ASPIRE/.venv/bin/python ./run_demo.sh
#
# Optional: SEED (default 0), PROGRAM (default examples/cerulion/pick_cube.py,
# relative to ASPIRE_DIR), OUT (a new directory; default runs/<timestamp>).
#
# Exit status:
#   0  the episode succeeded, the receipt chain holds and the replay matched
#   2  a prerequisite is missing (tool, directory, interpreter) or OUT is not new
#   3  the graph did not start recording, or did not finalize the bag
#   4  the ASPIRE episode failed (program error or task not solved)
#   5  the receipt chain is missing, empty or broken
#   6  the replay did not match (see OUT/replay.log)

set -euo pipefail

die() { echo "run_demo: $2" >&2; exit "$1"; }

WS="$(cd "$(dirname "$0")" && pwd)"
[ -n "${ASPIRE_DIR:-}" ] || die 2 "set ASPIRE_DIR to an ASPIRE checkout with the cerulion embodiment"
[ -d "$ASPIRE_DIR" ] || die 2 "ASPIRE_DIR $ASPIRE_DIR is not a directory"
ASPIRE_DIR="$(cd "$ASPIRE_DIR" && pwd)"
ASPIRE_PYTHON="${ASPIRE_PYTHON:-python3}"
case "$ASPIRE_PYTHON" in
  */*) ASPIRE_PYTHON="$(cd "$(dirname "$ASPIRE_PYTHON")" && pwd)/$(basename "$ASPIRE_PYTHON")" ;;
esac
"$ASPIRE_PYTHON" -c 'import cap_harness.cerulion' 2>/dev/null \
  || die 2 "$ASPIRE_PYTHON cannot import cap_harness.cerulion"
SEED="${SEED:-0}"
PROGRAM="${PROGRAM:-examples/cerulion/pick_cube.py}"
OUT="${OUT:-$WS/runs/$(date -u +%Y%m%dT%H%M%SZ)}"
if [ -e "$OUT" ] && [ -n "$(ls -A "$OUT" 2>/dev/null)" ]; then
  die 2 "$OUT is not empty (an earlier run's bag, receipts or e-stop latch); pick a new OUT"
fi
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"

for tool in cerulion cerulion-rcpd; do
  command -v "$tool" >/dev/null || die 2 "$tool is not on PATH"
done

cd "$WS"
echo "== build arm_sim"
cerulion node build arm_sim --release

echo "== start the graph, recording to $OUT/recordings"
# A non-interactive shell starts background jobs with SIGINT ignored, and the
# graph run stops (and its recorder finalizes the bag) only on SIGINT; the
# exec shim puts SIGINT back to its default before running the graph.
"$ASPIRE_PYTHON" -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
  cerulion graph run aspire_arm --release --record="$OUT/recordings" </dev/null >"$OUT/graph.log" 2>&1 &
GRAPH_PID=$!

# Stop the graph with SIGINT; escalate to SIGTERM, then SIGKILL, if it has
# not exited 30 s and 40 s later.
stop_graph() {
  kill -0 "$GRAPH_PID" 2>/dev/null || return 0
  kill -INT "$GRAPH_PID" 2>/dev/null || true
  for _ in $(seq 1 300); do
    kill -0 "$GRAPH_PID" 2>/dev/null || break
    sleep 0.1
  done
  if kill -0 "$GRAPH_PID" 2>/dev/null; then
    echo "run_demo: the graph ignored SIGINT for 30 s; sending SIGTERM" >&2
    kill -TERM "$GRAPH_PID" 2>/dev/null || true
    sleep 10
    kill -KILL "$GRAPH_PID" 2>/dev/null || true
  fi
  wait "$GRAPH_PID" 2>/dev/null || true
}
trap stop_graph EXIT

BAG=""
for _ in $(seq 1 240); do
  BAG="$(ls "$OUT"/recordings/*.mcap 2>/dev/null | head -n 1 || true)"
  [ -n "$BAG" ] && break
  kill -0 "$GRAPH_PID" 2>/dev/null || die 3 "the graph exited; see $OUT/graph.log"
  sleep 0.5
done
[ -n "$BAG" ] || die 3 "no bag after 120 s; see $OUT/graph.log"
# The bag file appears once the recorder's channel set is fixed, /rcp/calls
# included. The one-second margin before the first call is not measured to
# be needed; it costs a second.
sleep 1

echo "== ASPIRE: $PROGRAM, seed $SEED"
set +e
(
  cd "$ASPIRE_DIR"
  "$ASPIRE_PYTHON" -m cap_harness.cli run --benchmark cerulion --suite cerulion --task-id 0 \
    --seed "$SEED" --program "$PROGRAM" \
    --rcpd-manifest "$WS/rcp.yaml" --rcpd-receipts "$OUT/receipts.jsonl" \
    --output-root "$OUT/aspire" --camera-width 512 --camera-height 256
) >"$OUT/aspire.log" 2>&1
ASPIRE_STATUS=$?
set -e
grep -E '^(run:|program_ok=)' "$OUT/aspire.log" || true

echo "== stop the graph"
stop_graph
trap - EXIT
grep -E 'recording written to' "$OUT/graph.log" || die 3 "the graph did not finalize the bag; see $OUT/graph.log"

[ "$ASPIRE_STATUS" -eq 0 ] && grep -q 'task_success=True' "$OUT/aspire.log" \
  || die 4 "the episode failed (exit $ASPIRE_STATUS); see $OUT/aspire.log"

echo "== receipt chain"
cerulion-rcpd --verify-receipts "$OUT/receipts.jsonl" || die 5 "the receipt chain check failed"

echo "== replay with no agent: cerulion bag play --resim all --verify"
set +e
cerulion bag play "$BAG" --resim all --verify --report "$OUT/replay.json" >"$OUT/replay.log" 2>&1
REPLAY_STATUS=$?
set -e
grep -E '^replay (PASS|FAIL)' "$OUT/replay.log" || tail -n 20 "$OUT/replay.log"
[ "$REPLAY_STATUS" -eq 0 ] || die 6 "replay exit $REPLAY_STATUS; see $OUT/replay.log"
echo "== done: $OUT"
