#!/bin/sh
# SPDX-License-Identifier: AGPL-3.0-only
# Start the robot's vision service in the background and wait until it answers.
#   start.sh EQUIPMENT.json [PORT] [RUN_DIR] [FRAMES_ROOT]
# EQUIPMENT.json  the installation's equipment file ("" for a robot with no vision model: level absent)
# PORT            default 8791 (bound to 127.0.0.1 only)
# RUN_DIR         where eyes.pid and eyes.log go (default: ./eyes_run)
# FRAMES_ROOT     the folder a request's image_path may name a file under (default: none, so every image_path is
#                 refused and pictures travel in the request); also read from the environment as EYES_FRAMES_ROOT
# Env: EYES_PYTHON (the Python that has the backends' packages, default python3), EYES_WAIT_S (default 180).
# The tree that holds the rcpx package is found from this file's place.
HERE=$(cd "$(dirname "$0")" && pwd)
TREE=$(cd "$HERE/../.." && pwd)
EQ=${1-}; PORT=${2:-8791}; RUN=${3:-./eyes_run}; FRAMES=${4:-${EYES_FRAMES_ROOT-}}
PY=${EYES_PYTHON:-python3}; WAIT=${EYES_WAIT_S:-180}
mkdir -p "$RUN" || exit 1
RUN=$(cd "$RUN" && pwd)
if [ -f "$RUN/eyes.pid" ] && kill -0 "$(cat "$RUN/eyes.pid")" 2>/dev/null; then
  echo "eyes: already running, pid $(cat "$RUN/eyes.pid")"; exit 0
fi
if [ -n "$FRAMES" ]; then
  [ -d "$FRAMES" ] || { echo "eyes: the frames root is not a folder: $FRAMES"; exit 1; }
  FRAMES=$(cd "$FRAMES" && pwd)
fi
cd "$TREE" || exit 1
if [ -n "$EQ" ]; then set -- --equipment "$EQ"; else set --; fi
if [ -n "$FRAMES" ]; then set -- "$@" --frames-root "$FRAMES"; fi
nohup "$PY" -m rcpx.eyes_service.server "$@" --port "$PORT" --pid-file "$RUN/eyes.pid" >> "$RUN/eyes.log" 2>&1 &
PID=$!
i=0
while [ "$i" -lt "$WAIT" ]; do
  if ! kill -0 "$PID" 2>/dev/null; then echo "eyes: exited while starting; see $RUN/eyes.log"; exit 1; fi
  if "$PY" -c "import json,sys,urllib.request; print('eyes: up, level', json.load(urllib.request.urlopen('http://127.0.0.1:$PORT/health', timeout=2))['level'])" 2>/dev/null; then
    echo "eyes: pid $PID, port $PORT, log $RUN/eyes.log"; exit 0
  fi
  sleep 1; i=$((i + 1))
done
echo "eyes: not answering after $WAIT s (pid $PID still loading?); see $RUN/eyes.log"; exit 1
