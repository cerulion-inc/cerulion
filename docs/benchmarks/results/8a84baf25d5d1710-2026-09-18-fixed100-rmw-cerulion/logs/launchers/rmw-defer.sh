#!/usr/bin/env bash
set -uo pipefail
LANE=<BOXHOME>/lanes/rmw-latency-lane-20260918
cd "$LANE"; export DOCKER_BUILDKIT=1
docker build --build-arg ROS_DISTRO=jazzy --label com.cerulion.bench.repo_git_sha="$(git rev-parse HEAD)" \
    -t latency_bench:jazzy -f "$LANE/benches/latency/ros2/docker/Dockerfile" "$LANE" 2>&1 | tail -1
echo "=== image: $(docker images --no-trunc --format "{{.ID}}" latency_bench:jazzy)"
rm -rf <BOXHOME>/lanes/defer
for SZ in 64 262144 1048576 4194304 16777216; do
  OUT=<BOXHOME>/lanes/defer/sz$SZ; mkdir -p "$OUT/raw"
  timeout 420 docker run --rm --shm-size=4g --cap-add SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 \
    --sysctl net.ipv4.ipfrag_high_thresh=134217728 \
    -v "$OUT/raw":/raw -v "$LANE":/work -e CER_RMW_REPO=/work \
    -v "$LANE/benches/latency/ros2/run_bench.sh":/bench/run_bench.sh:ro \
    -v "$LANE/benches/latency/ros2/configs":/bench/configs:ro \
    -v "$LANE/benches/latency/ros2/verify_shm.sh":/bench/verify_shm.sh:ro \
    -e CER_BENCH_RAW_DUMP_DIR=/raw -e CER_BENCH_RAW_NAME=d -e CER_BENCH_PACING=fixed100 \
    -e SIZES=$SZ -e CER_BENCH_TARGET_SAMPLES=2000 -e CER_BENCH_WARMUP=100 \
    -e TARGET_SAMPLES=2000 -e WARMUP=100 -e CER_BENCH_QOS=be1 -e RMWS=cerulion \
    -e SHM_MODE=shm -e RECV_PATH=loan -e CER_BENCH_MSG=pod -e CHRT_MODE=off \
    -e WITH_CHRT=0 -e WITH_SHM=0 -e BENCH_CELL_TIMEOUT_S=300 \
    -e CER_BENCH_PONG_ATTRIB=1 -e CER_BENCH_PONG_GAP_OUT=/raw/rec.bin \
    -e CER_BENCH_LAT_CPU_PROBE=1 -e CER_BENCH_LAT_CPU_OUT=/raw/latcpu.i32 \
    latency_bench:jazzy > "$OUT/cell.log" 2>&1
  IN=$(grep -c "borrowed INLINE" "$OUT/cell.log" 2>/dev/null || echo 0)
  echo "sz=$SZ rtt=$(stat -c%s $OUT/raw/d_$SZ.bin 2>/dev/null) inline_warn=$IN"
done
grep -h -o "echo_rule=[a-z_]* prefetch_depth=[0-9]* refill_defer_us=[0-9]*" <BOXHOME>/lanes/defer/sz1048576/raw/*_node.log | sort -u
echo "=== DEFER SMOKE FINISHED ==="
