#!/usr/bin/env bash
# Start (or reuse) the long-lived Lyrical + CUDA container on the Jetson.
#
#   container/run.sh            start the container if it is not running
#   container/run.sh exec CMD   run CMD inside it (bash -lc, ROS sourced)
#
# Mounts: the host CUDA 12.6 toolkit read-only (the L4T nvidia runtime mounts
# only the driver), the overlay workspace, and this fixtures directory.
# The container has no network namespace of its own beyond loopback
# (--network none): every pub/sub in the fixtures runs inside it, and DDS
# discovery is held to localhost. Override with S2_NETWORK=host if a run
# needs the host network (S1's cargo fetch, for example).
set -euo pipefail
IMAGE=${S2_IMAGE:-cerulion/lyrical-cuda:s2}
NAME=${S2_CONTAINER:-s2-lyrical}
WS=${S2_WS:-$HOME/gpu-s2/ws}
FIX=${S2_FIXTURES:-$(cd "$(dirname "$0")/.." && pwd)}
NET=${S2_NETWORK:-none}
mkdir -p "$WS"

if ! docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  docker rm -f "$NAME" >/dev/null 2>&1 || true
  docker run -d --name "$NAME" --runtime nvidia --network "$NET" \
    --shm-size 2g --ulimit nofile=65536:65536 --cap-add SYS_PTRACE \
    -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    -v /usr/local/cuda-12.6:/usr/local/cuda-12.6:ro \
    -v "$WS":/opt/s2/ws -v "$FIX":/opt/s2/fixtures \
    "$IMAGE" sleep infinity >/dev/null
fi

if [ "${1:-}" = exec ]; then
  shift
  exec docker exec -i "$NAME" bash -lc "source /opt/ros/lyrical/setup.bash; \
    [ -f /opt/s2/ws/install/setup.bash ] && source /opt/s2/ws/install/setup.bash; $*"
fi
