#!/usr/bin/env bash
# run_demo.sh — one-command host driver for the MoveIt hero demo.
#
# Builds the image, then runs the unmodified MoveIt Panda demo over
# RMW_IMPLEMENTATION=rmw_cerulion inside the container, mounting the repo so
# rmw_cerulion is built from source. Run from the repo root:
#
#   bash examples/moveit_hero/run_demo.sh            # build + run
#   bash examples/moveit_hero/run_demo.sh --capture  # also keep move_group's own
#                                                 # log, for hero-media capture
#   CER_DEMO_NO_BUILD=1 bash examples/moveit_hero/run_demo.sh   # skip docker build
#
# Exit code is the demo's: 0 = move_group came up on rmw_cerulion and both the
# OMPL plan and the pilz determinism check passed.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IMAGE="${CER_DEMO_IMAGE:-cerulion-moveit-hero}"
CAPTURE=0
[ "${1:-}" = "--capture" ] && CAPTURE=1

cd "$REPO_ROOT"

if [ "${CER_DEMO_NO_BUILD:-0}" != "1" ]; then
    echo "=== building $IMAGE ==="
    docker build -f examples/moveit_hero/Dockerfile -t "$IMAGE" examples/moveit_hero
fi

# --shm-size=1g:      64M default starves iceoryx2 segments
# --ulimit nofile:    move_group maps hundreds of SHM segments; 1024 exhausts fds
echo "=== running MoveIt hero demo (RMW_IMPLEMENTATION=rmw_cerulion) ==="
docker run --rm \
    --shm-size=1g \
    --ulimit nofile=524288 \
    -e CER_DEMO_CAPTURE="$CAPTURE" \
    -v "$REPO_ROOT":/work -w /work \
    "$IMAGE" \
    bash examples/moveit_hero/entrypoint.sh
