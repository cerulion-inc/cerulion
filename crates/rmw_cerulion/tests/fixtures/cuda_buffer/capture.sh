#!/usr/bin/env bash
# Regenerate every fixture in golden/ on a Jetson with CUDA (the capture box:
# L4T R36, CUDA 12.6 at /usr/local/cuda-12.6, docker + nvidia runtime).
#
#   ./capture.sh [stage ...]       stages: image overlay env samples descriptor
#                                  ipc fallback latency collect (default: all)
#
# Run it from this directory on the box. Each stage writes under
# $S2_WS/out/<stage> inside the container's workspace mount; `collect` copies
# the small, reviewable outputs into golden/. The box lock is taken per
# stage, waiting for it.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
export S2_WS=${S2_WS:-$HOME/gpu-s2/ws}
LOCK=${S2_LOCK:-/tmp/cerulion-box.lock}
RUN="$HERE/container/run.sh"
OUT=/opt/s2/ws/out
STAGES=${*:-image overlay env samples descriptor ipc fallback latency collect}

locked() { flock "$LOCK" "$@"; }

for stage in $STAGES; do
  echo "== $stage $(date -u +%FT%TZ)"
  case $stage in
    image)
      locked docker build --network host -t "${S2_IMAGE:-cerulion/lyrical-cuda:s2}" "$HERE/container"
      docker rm -f "${S2_CONTAINER:-s2-lyrical}" >/dev/null 2>&1 || true
      "$RUN" ;;
    overlay)
      locked "$RUN" exec bash /opt/s2/fixtures/container/build-overlay.sh ;;
    env)
      tmp=$(mktemp)
      {
        echo "l4t: $(head -1 /etc/nv_tegra_release)"
        echo "kernel: $(uname -r)"
        echo "docker: $(docker version --format '{{.Server.Version}}')"
        echo "image: $(docker image inspect "${S2_IMAGE:-cerulion/lyrical-cuda:s2}" --format '{{.Id}}')"
        echo "cuda_toolkit: $(/usr/local/cuda-12.6/bin/nvcc --version | tail -1)"
        echo "nvpmodel: $(nvpmodel -q 2>/dev/null | tr '\n' ' ' || echo unreadable)"
        echo "cpu_governor: $(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | sort | uniq -c | tr -s ' ' | tr '\n' ';')"
        for g in /sys/class/devfreq/*gpu*; do
          [ -e "$g/cur_freq" ] && echo "gpu_devfreq: $(basename "$g") cur=$(cat "$g/cur_freq") min=$(cat "$g/min_freq") max=$(cat "$g/max_freq") gov=$(cat "$g/governor")"
        done
        echo "load: $(cut -d' ' -f1-3 /proc/loadavg)"
      } > "$tmp"
      "$RUN" exec "mkdir -p $OUT/env && cp /opt/s2/ros-lyrical-packages.tsv $OUT/env/ && \
        git -C /opt/s2/src/rosidl_buffer_backends rev-parse HEAD > $OUT/env/rosidl_buffer_backends.commit"
      docker cp "$tmp" "${S2_CONTAINER:-s2-lyrical}:$OUT/env/host.txt"
      rm -f "$tmp" ;;
    samples)
      locked "$RUN" exec bash /opt/s2/fixtures/scripts/samples.sh "$OUT" ;;
    descriptor)
      locked "$RUN" exec "mkdir -p $OUT/descriptor && \
        /opt/s2/ws/install/lib/cuda_buffer_s2_probe/descriptor_cdr_probe $OUT/descriptor \
          > $OUT/descriptor/probe.json 2> $OUT/descriptor/probe.stderr && \
        python3 /opt/s2/fixtures/scripts/decode_cdr.py $OUT/descriptor $OUT/descriptor/decoded.json" ;;
    ipc)
      locked "$RUN" exec python3 /opt/s2/fixtures/scripts/ipc_lifecycle.py \
        /opt/s2/ws/install/lib/cuda_buffer_s2_probe "$OUT/ipc" ;;
    fallback)
      locked "$RUN" exec bash /opt/s2/fixtures/scripts/fallback_wire.sh "$OUT" ;;
    latency)
      # Two runs: latency/ stamps before the producer's GPU write (enqueued),
      # latency_ready/ after it (golden/latency/ready/). run.sh exec does not
      # forward host env, so the ready run's overrides are in the command.
      locked "$RUN" exec bash /opt/s2/fixtures/scripts/latency.sh "$OUT"
      locked "$RUN" exec "S2_STAMP=ready S2_MODES='inter intra proc1_rmw' \
        bash /opt/s2/fixtures/scripts/latency.sh $OUT" ;;
    collect)
      "$RUN" exec python3 /opt/s2/fixtures/scripts/collect.py "$OUT" /opt/s2/fixtures/golden ;;
    *) echo "unknown stage $stage" >&2; exit 2 ;;
  esac
done
