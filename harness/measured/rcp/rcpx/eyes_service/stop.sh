#!/bin/sh
# SPDX-License-Identifier: AGPL-3.0-only
# Stop the robot's vision service: by the pid in RUN_DIR/eyes.pid only (never by name, never a group).
#   stop.sh [RUN_DIR]
RUN=${1:-./eyes_run}
F="$RUN/eyes.pid"
[ -f "$F" ] || { echo "eyes: no pid file in $RUN (not running?)"; exit 0; }
PID=$(cat "$F")
if kill -0 "$PID" 2>/dev/null; then
  kill "$PID"
  i=0
  while kill -0 "$PID" 2>/dev/null && [ "$i" -lt 20 ]; do sleep 0.5; i=$((i + 1)); done
  if kill -0 "$PID" 2>/dev/null; then echo "eyes: pid $PID did not stop on TERM"; exit 1; fi
  echo "eyes: stopped pid $PID"
else
  echo "eyes: pid $PID was not running"
fi
rm -f "$F"
